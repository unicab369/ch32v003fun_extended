

// #include "stdio.h"
// #include "TestClass.h"

// TestClass testClass;

// void TestClass::setup() {
//    GPIO_port_enable(GPIO_port_D);

//    // GPIO_pinMode(GPIOv_from_PORT_PIN(GPIO_port_D, 4), GPIO_pinMode_O_pushPull, GPIO_Speed_10MHz);   // GPIO D0 Push-Pull
//    GPIO_pinMode(GPIOv_from_PORT_PIN(GPIO_port_D, 6), GPIO_pinMode_O_pushPull, GPIO_Speed_10MHz);   // GPIO D0 Push-Pull

//    GPIO_port_enable(GPIO_port_C);
//    GPIO_pinMode(GPIOv_from_PORT_PIN(GPIO_port_C, 1), GPIO_pinMode_I_pullUp, GPIO_Speed_In);   // GPIO D0 Push-Pull
//    GPIO_pinMode(GPIOv_from_PORT_PIN(GPIO_port_C, 2), GPIO_pinMode_I_pullUp, GPIO_Speed_In);   // GPIO D0 Push-Pull
//    GPIO_pinMode(GPIOv_from_PORT_PIN(GPIO_port_C, 4), GPIO_pinMode_I_pullUp, GPIO_Speed_In);   // GPIO D0 Push-Pull
// }

// void TestClass::run() {
//    // GPIO_digitalWrite(GPIOv_from_PORT_PIN(GPIO_port_D, 6), high);
//    // GPIO_digitalWrite(GPIOv_from_PORT_PIN(GPIO_port_C, 1), high);
//    // GPIO_digitalWrite(GPIOv_from_PORT_PIN(GPIO_port_C, 2), high);
//    // GPIO_digitalWrite(GPIOv_from_PORT_PIN(GPIO_port_C, 4), high);
//    // Delay_Ms(200);

//    // GPIO_digitalWrite(GPIOv_from_PORT_PIN(GPIO_port_D, 6), low);
//    // GPIO_digitalWrite(GPIOv_from_PORT_PIN(GPIO_port_C, 1), low);
//    // GPIO_digitalWrite(GPIOv_from_PORT_PIN(GPIO_port_C, 2), low);
//    // GPIO_digitalWrite(GPIOv_from_PORT_PIN(GPIO_port_C, 4), low);
//    // Delay_Ms(200);

//    uint8_t isOn1 = !GPIO_digitalRead(GPIOv_from_PORT_PIN(GPIO_port_C, 1));
//    uint8_t isOn2 = !GPIO_digitalRead(GPIOv_from_PORT_PIN(GPIO_port_C, 2));
//    uint8_t isOn4 = !GPIO_digitalRead(GPIOv_from_PORT_PIN(GPIO_port_C, 4));

//    if (isOn1 || isOn2 || isOn4) {
//       GPIO_digitalWrite(GPIOv_from_PORT_PIN(GPIO_port_D, 6), high);
//    } else {
//       GPIO_digitalWrite(GPIOv_from_PORT_PIN(GPIO_port_D, 6), low);
//    }
//    // enum lowhigh value = isOn ? high : low;
// }