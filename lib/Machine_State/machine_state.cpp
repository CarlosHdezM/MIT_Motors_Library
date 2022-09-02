#include "machine_state.h"

void print_menu()
{
    Serial.print("\n*** Escoge una opcion: ***\n");
    Serial.print(INPUT_ENABLE_MOTORS - '0'); Serial.print(") Habilitar Motores \n");
    Serial.print(INPUT_DISABLE_MOTORS - '0'); Serial.print(") Deshabilitar Motores \n");
    Serial.print(INPUT_SET_TORQUE_ZERO - '0'); Serial.print(") Enviar torque cero \n");
    Serial.print(INPUT_READ_MOTOR_RESPONSE - '0'); Serial.print(") Leer respuesta del motor \n");
    Serial.print(INPUT_REQUEST_POSITION - '0'); Serial.print(") Solicitar posicion del motor **Uso Exclusivo para motores RMD**\n");
    Serial.print(INPUT_SET_TORQUE - '0'); Serial.print(") Establecer torque a motores \n");
    Serial.print(INPUT_SET_TORQUE_AND_READ - '0'); Serial.print(") Enviar y leer continuamente \n");
    Serial.print(INPUT_SET_POS_ORIGIN - '0'); Serial.print(") Establecer position actual como Origen\n");
    Serial.print(INPUT_SET_POS_ZERO - '0'); Serial.print(") Establecer position actual como Zero \n");

}
