# Wrapper Odrive S1/PRO
napajeni  24 V  
nastaven bitrate 500 000  

## Aktivace CAN interface
Pred spustenim kodu musi se nakonfigurovat port pro prijimac.   

```
sudo ip link set can0 type can bitrate 500000  
sudo ip link set up can0  
```

Da se pozmenit nastaveni v `sudo nano /etc/network/interfaces`.
```
auto can0
  iface can0 inet manual
  pre-up /sbin/ip link set can0 type can bitrate 500000
  up /sbin/ifconfig can0 up
  down /sbin/ifconfig can0 down 
```

Nebo, se musi pridat uzivateli moznost spoustet sudo prikazy bez hesla  

`echo "$USER ALL=(ALL:ALL) NOPASSWD: ALL" | sudo tee "/etc/sudoers.d/dont-prompt-$USER-for-sudo-password"`



## Požadavky před spuštěním
Na Usb2CAN tripele má být spušten adapter _triple_.
```sh ./triple.sh ```
Dá se ověřit, že nstavení proběhlo správně přes ```candump canX```, kde ```X``` je cislo portu, na který je can na převodníku připojen.

## Specifikace C++ interface pro ODrive
Dokumentace pro ODrive, kde lze dohledat všechny následující parametry zmíněné v tomto dokumentu
https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html

Jako sběrnice se bude použit CAN.
https://docs.odriverobotics.com/v/latest/can-protocol.html

Dostupné implementace
https://github.com/belovictor/odrive_can_ros_driver

Základní potřebné informace funkcionality dostupné z C++, všechny ostatní parametry budou nastavené staticky pomocí ODrive GUI/webpage.

- __Informace vyčítané pouze při startu__
- *Vyčítat opakovaně a udržovat aktuální hodnotu*
- `Potřeba nastavovat/volat opakovaně`

**Informace o ODrive**
- [ ] __serial_number__ - neni soucasti CAN protokolu
- [ ] __hw_version_major__- neni soucasti
- [ ] __hw_version_minor__- neni soucasti
- [ ] __hw_version_variant__- neni soucasti
- [ ] __fw_version_major__- neni soucasti
- [ ] __fw_version_minor__ - neni soucasti
- [ ] __fw_version_revision__- neni soucasti
- [ ] __commit_hash__ - neni soucasti
- [ ] `reboot`

**CAN**
- [ ] *Can.error*
- [ ] *Can.n_restarts*
- [ ] *Can.n_rx*

**Chybové stavy ODrive**
- [ ] *Issues* - Chyby celého ODrive
- [ ] *Misconfigured* - Signalize že je ODrive správně nakonfigurován- neni soucasti
- [ ] `Clear_errors` - Smazání chyb

**Spotřeba ODrive**
- [ ] *Ibus* - Proud odebíraný ODrivem
- [ ] *Vbus_voltage* - Vstupní napětí ODrive

**Ovládání stav**
- [ ] *Axis.active_errors* - Aktivní chyby motoru
- [ ] *Axis.disarm_reason* - Chyby které způsobily zastavení motoru- neni soucasti
- [ ] *Axis.last_drv_fault*  - Poslední chyba driveru- neni soucasti
- [ ] *Axis.AxisState* - aktuální stav driveru (idle, closed_loop_ctrl,...)

**Nastavení pozice motoru**
- [ ] `Axis.set_abs_pos` - Nastavení absolutní pozice motoru

**Nastavení regulačního módu**
- [ ] `Axis.controller.config.control_mode` - Nastavení regulačního modu
- [ ] `Axis.controller.config.input_mode` - Nastavení typu regulační modu

**Rychlostní regulační mód**
- [ ] `Axis.controller.config.vel_ramp_rate` - Nastaveni zrychlení rychlostní rampy
- [ ] `Axis.controller.input_vel` - Nastavení rychlostního setpointu

**Poziční regulační mód**
- [ ] `Axis.trap_traj.config.vel_limit` - Maximální rychlosti pro poziční regulaci
- [ ] `Axis.trap_traj.config.accel_limit` - Maximální zrychlení poziční regulaci
- [ ] `Axis.trap_traj.config.decel_limit` - Maximální zpomalení poziční regulaci
- [ ] `Axis.controller.input_pos` - Nastavení pozičního setpointu
- [ ] *Axis.controller.trajectory_done* - Nastaveni pozicního setpointu

**Momentový regulační mód**
- [ ] `Axis.controller.config.torque_ramp_rate` 
- [ ] `Axis.controller.input_torque` - Nastavení požadovaného momentu

**Výkon motoru**
- [ ] *Axis.controller.mechanical_power* - Mechanický výkon motoru
- [ ] *Axis.controller.electrical_power* - Elektircký výkon motoru

**Proudy fázemi**
- [ ] *AlphaBetaFrameController.current_meas_phA* - Změřený proud tekoucí fází A- neni soucasti
- [ ] *AlphaBetaFrameController.current_meas_phB* - Změřený proud tekoucí fází B- neni soucasti
- [ ] *AlphaBetaFrameController.current_meas_phC* - Změřený proud tekoucí fází C- neni soucasti
- [ ] *AlphaBetaFrameController.current_meas_status_phA* - Status měřeného proudu - neni soucastifáze A
- [ ] *AlphaBetaFrameController.current_meas_status_phB* - Status měřeného proudu - neni soucastifáze B
- [ ] *AlphaBetaFrameController.current_meas_status_phC* - Status měřeného proudu - neni soucastifáze C
- [ ] *AlphaBetaFrameController.I_bus* - Proudový odběr ODrivu
- [ ] *AlphaBetaFrameController.Ialpha_measured* - Měřený alpha proud
- [ ] *AlphaBetaFrameController.Ibeta_measured* - Měřený beta proud
- [ ] *AlphaBetaFrameController.power* - Výkon dodávaný motoru ODrivem

**Sledovač stavu motoru**
- [ ] *Mapper.status* - Status sledovače stavu motoru- neni soucasti
- [ ] *Mapper.pos_rel* - Poloha motoru- neni soucasti
- [ ] *Mapper.pos_abs* - Absolutní poloha motoru- neni soucasti
- [ ] *Mapper.vel* - Rychlost motoru- neni soucasti

**Teplota na ODriveru**
- [ ] *OnboardThermistorCurrentLimiter.temperature*

**Teplota na externím senzoru**
- [ ] *OffboardThermistorCurrentLimiter.temperature*

**Odpor na pálení vygenerovaného proudu**
- [ ] *BrakeResistor.current_meas*- neni soucasti
- [ ] *BrakeResistor.current*- neni soucasti
- [ ] *BrakeResistor.chopper_temp*- neni soucasti


## “SimpleCAN” protocol:
### Prikazy
Estop
Start_Anticogging
Reboot
ClearErrors 
### Ctene parametry
Hearthbeat  
- axis.error_
- axis.current_state_
- axis.controller_.trajectory_done_

Get_Motor_Error  
-axis.motor_.error_

Get_Encoder_Error  
- axsi.encoder_.error_

Get_Sensorledd_Erorr  
- axis.sensorless_estimator_.error_

Get_Encoder_Estimates  
- axis.encoder_.pos_estimate_
- axis.encoder_.vel_estimate_

Get_Encoder_Counts  
- axis.encoder_.shadow_count_
- axis.encoder_.count_in_cpr_

Get_Iq  
- axis.motor_.current_control_.Idq_setpoint_
- axis.motor_.current_control_.Iq_measured_

Get_Sensorless_Estimates  
- axis.sensorless_estimator_.pll_pos_
- axis.sensorless_estimator_.vel_estimate_

Get_Vbus_Voltage  
- vbus_voltage
- v popisu protokolu sice pisou ze tam je i current v kodu ani v DBC neni
### Nastavovane parametry 
Set_Axis_Node_Id      
- axis.config_.can.node_id  

Set_Axis_State    
- axis.requested_state_  

Set_Controller_Mode  
- axis.controller_.config_.control_mode   
- axis.controller_.config_.input_mode   

Set_Input_Pos     
- axis.controller_.set_input_pos_and_steps()
- axis.controller_.input_vel_
- axis.controller_.input_torque_

Set_Input_Vel   
- axis.controller_.input_vel_
- axis.controller_.input_torque_

Set_Input_Torque  
- axis.controller_.input_torque_ 
 
Set_Limits  
- axis.controller_.config_.vel_limit
- axis.motor_.config_.current_lim

Set_Traj_Vel_Limits  
- axis.trap_traj_.config_.vel_limit

Set_Traj_Accel_Limits  
- axis.trap_traj_.config_.accel_limit
- axis.trap_traj_.config_.decel_limit

Set_Traj_Inertia  
- axis.controller_.config_.inertia

Set_Linear_Count  
- axis.encoder_.set_linear_count()

Set_Pos_Gain  
- axis.controller_.config_.pos_gain

Set_Vel_gains  
- axis.controller_.config_.vel_gain
- axis.controller_.config_.vel_integrator_gain



