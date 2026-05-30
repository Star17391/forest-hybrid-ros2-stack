# Deploy (Raspberry Pi / campo)

Ficheiros **systemd**, ambientes Conda/ROS e procedimentos de instalação no robô — a completar à medida que o *hardware* final estiver fixo.

## Princípios

- O robô de produção deve **source** o workspace deste repositório após `colcon build` (`install/setup.bash`), tal como no PC de desenvolvimento.
- Serviços systemd devem invocar um *wrapper* que faz `source /opt/ros/$ROS_DISTRO/setup.bash` e `source /caminho/instalado/install/setup.bash` antes de `ros2 launch ...`.

## Comandos úteis (esboço)

```bash
# No robô, após clonar e compilar o workspace
sudo mkdir -p /etc/ros/hybrid
sudo cp etc/hybrid.env /etc/ros/hybrid/   # quando existir
sudo systemctl enable --now forest-hybrid-bringup.service  # nome exemplificativo
```

Detalhes concretos (unidades, utilizador, `ROS_DOMAIN_ID`, rede CycloneDDS) serão adicionados quando o *fleet* for definido.

## Relação com simulação

Simulador e *bridges* seguem o contrato em [../docs/LAYER_CONTRACTS.md](../docs/LAYER_CONTRACTS.md); o *deploy* de campo usa os **mesmos nomes de tópicos**, com `use_sim_time:=false`.
