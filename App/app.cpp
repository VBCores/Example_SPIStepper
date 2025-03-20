#include "app.h"

#include <memory>

#include "tim.h"
#include "i2c.h"
#include "fdcan.h"
#include "spi.h"

#include <cyphal/node/node_info_handler.h>
#include <cyphal/node/registers_handler.hpp>
#include <cyphal/providers/G4CAN.h>

#include <uavcan/node/Mode_1_0.h>
#include <uavcan/si/unit/angle/Scalar_1_0.h>
#include <uavcan/primitive/scalar/Natural32_1_0.h>

#include <voltbro/eeprom/eeprom.hpp>
#include <voltbro/motors/stepper/stepper_spi.hpp>

EEPROM eeprom(&hi2c4);

// Example of using inheritance of the motor class to customize logic
class CustomStepper: public StepperMotorSPI {
public:
    using StepperMotorSPI::StepperMotorSPI; // inherit constructors

    /* Override this method to send custom configuration over SPI
    HAL_StatusTypeDef send_config() override {
        // SPI CONFIG HERE
        // SEE Drivers/libvoltbro/voltbro/motors/stepper/stepper_spi.hpp:77 as example
    }
    */

    /* Override this method to customize setting of target value for motion controller
        * By default, sets position
        * SEE Drivers/libvoltbro/voltbro/motors/stepper/stepper_spi.hpp:109 as example
    void set_target(uint32_t value) override {
        // SEE TMC510 docs for documentation on SPI registers
    }
    */
};

CustomStepper motor(
    StepperSPIConfig{
        .direction = GpioPin(REFL_DIR_GPIO_Port, REFL_DIR_Pin),
        .sd_mode = GpioPin(SD_MODE_GPIO_Port, SD_MODE_Pin),
        .spi_mode = GpioPin(SPI_MODE_GPIO_Port, SPI_MODE_Pin),
        .cfg4 = GpioPin(CFG4_GPIO_Port, CFG4_Pin),
        .cfg5 = GpioPin(CFG5_GPIO_Port, CFG5_Pin),
        .cfg6 = GpioPin(CFG6_GPIO_Port, CFG6_Pin),
        .spi_ss = GpioPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin),
        .spi = &hspi1,
        .step_channel = TIM_CHANNEL_1,
        .timer = &htim1
    },
    GpioPin(DRV_EN_GPIO_Port, DRV_EN_Pin)
);

[[noreturn]] void app() {
    start_timers();
    start_cyphal();

    while (!eeprom.is_connected()) {
        eeprom.delay();
    }
    eeprom.delay();

    motor.start();
    HAL_IMPORTANT(motor.init());

    set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);

    while(true) {
        cyphal_loop();
        motor.update_position();
    }
}

TYPE_ALIAS(Natural32, uavcan_primitive_scalar_Natural32_1_0)
static constexpr CanardPortID POSITION_PORT = 6885;

void in_loop_reporting(millis current_t) {
    static millis report_time = 0;
    EACH_N(current_t, report_time, 50, {
        Natural32::Type pos_msg = {};
        pos_msg.value = motor.get_position();
        static CanardTransferID pos_transfer_id = 0;
        get_interface()->send_msg<Natural32>(&pos_msg, POSITION_PORT, &pos_transfer_id);
    })
}

class PositionSub: public AbstractSubscription<Natural32> {
public:
    PositionSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Natural32>(interface, port_id) {};
    void handler(const Natural32::Type& msg, CanardRxTransfer* _) override {
        motor.set_target(msg.value);
    }
};

ReservedObject<NodeInfoReader> node_info_reader;
ReservedObject<RegistersHandler<1>> registers_handler;
ReservedObject<PositionSub> pos_sub;

void setup_subscriptions() {
    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE
    );

    auto cyphal_interface = get_interface();
    const auto node_id = get_node_id();

    pos_sub.create(cyphal_interface, POSITION_PORT + node_id);
    node_info_reader.create(
        cyphal_interface,
        "org.voltbro.stepper",
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        0
    );
    registers_handler.create(
        std::array<RegisterDefinition, 1>{{
            {
                "motor.is_on",
                [](
                    const uavcan_register_Value_1_0& v_in,
                    uavcan_register_Value_1_0& v_out,
                    RegisterAccessResponse::Type& response
                ){
                    static bool value = false;
                    if (v_in._tag_ == 3) {
                        value = v_in.bit.value.bitpacked[0] == 1;
                    }
                    else {
                        // TODO: report error
                    }

                    motor.set_state(value);

                    response.persistent = true;
                    response._mutable = true;
                    v_out._tag_ = 3;
                    v_out.bit.value.bitpacked[0] = motor.is_on();
                    v_out.bit.value.count = 1;
                }
            }
        }},
        cyphal_interface
    );

    static FDCAN_FilterTypeDef sFilterConfig;
    uint32_t filter_index = 0;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        node_info_reader->make_filter(node_id)
    ))

    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        registers_handler->make_filter(node_id)
    ))

    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        pos_sub->make_filter(node_id)
    ))
}
