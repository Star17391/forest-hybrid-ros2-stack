# MAVLink (vendorizado)

Biblioteca C *header-only* gerada do dialeto `ardupilotmega` (que inclui `common`,
`standard`, `minimal` e dialetos auxiliares). Copiada de:

    ~/ardupilot/build/sitl/libraries/GCS_MAVLink/include/mavlink/v2.0

Vendorizada aqui para o pacote compilar sem dependências externas de runtime
(sem MAVSDK, sem libmavlink-dev, sem MAVROS). Inclui-se apenas
`ardupilotmega/mavlink.h` no código C++.

MAVLink é distribuído sob licença MIT (https://github.com/mavlink/mavlink).
Estes headers são gerados; não editar à mão. Para atualizar, recopiar a árvore
`v2.0` de um build recente do ArduPilot.
