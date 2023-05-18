# Wrapper Odrive S1/PRO
napajeni  24 V  
nastaven bitrate 500 000  

## Spusteni odrive
1. nastavit axis state na closed loop control (odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL)
2. spusteni v velocity regulation mode pres controller_input_vel = 10 (odrv0.axis0.controller.input_vel = 10)

## Aktivace CAN interface
Pred spustenim kodu musi se nakonfigurovat port pro prijimac.   

```
sudo ip link set can0 type can bitrate 500000  
sudo ip link set up can0  
sudo ifconfig can0 txqueuelen 1000
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

## Ztrata zprav na CAN
pokud se zacnou ztracet hromadne zpravy a vypisuje se write error, je treba zvysit periodu mezi jednotlivymi odeslani periodickych pozadavku

## Požadavky před spuštěním
Na Usb2CAN tripele má být spušten adapter _triple_.
```sh ./triple.sh ```
Dá se ověřit, že nstavení proběhlo správně přes ```candump canX```, kde ```X``` je cislo portu, na který je can na převodníku připojen.

## Nalezene bugy v odrive
- pri nastavovani Axis ID pomoci odrivetool si sice zapamatuje na jakem ID ma vysilat, ovsem do ulozeni a restartu odmita prijimat zpravy
- neodpovida na zpravu Get Version
- pri jakekoli chybe nepouzije standardni Can protokol, kdy se do CAN ID prida flag, ale odesla zpravu vyplenou nulami, coz odpovida Axis0_Get_version
- Get_Bus_Voltage_Current odpovida pouze na voltage
    - i kdyz ze seznamu se hodnota tvari jako unit32, jedna se o float
- Na stránkách je odkaz na DBC kód, který ale ovšem není aktuální. Tento dirver je naprogramován pro verzi 0.6.6 zdokumentovanou na strankach Odrive Pro Documentation

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
- Informace o verzi je nedostupna v soucasne implementaci simpleCAN od odrive

**Informace o ose**
- *AxisState* - aktuální stav driveru (idle, closed_loop_ctrl,...)
- *Trajectory done flag*
- *Procedure result*
- *set axis state*

**CAN**
- [ ] *Can.error*
- [ ] *Can.n_restarts*
- [ ] *Can.n_rx*

**Chybové stavy ODrive**
- *Active errors* - Chyby (asi celého) ODrive
- *Disarm reason* - 
- *Axis error* - Chyba osy
- *Controller error*
-  `Clear_errors` - Smazání chyb

**Spotřeba ODrive**
-  *bus current* - Proud odebíraný ODrivem, dle zkousky nefunkcni
-  *bus voltage* - Vstupní napětí ODrive, ve floatech
- *ADC voltage*

**Nastavení pozice motoru**
- `set absolute position` - Nastavení absolutní pozice motoru

**Nastavení limitů**
- `velocity limit`
- `current limit`

**Nastavení regulačního módu**
- `control_mode` - Nastavení regulačního modu
- `input_mode` - Nastavení typu regulační modu

**Rychlostní regulační mód**
- `input velocity` - Nastavení rychlostního setpointu
- `input torque feed forward`
- `velocity gain`
- `velocity integrator gain`

**Poziční regulační mód**
- `traj_vel_limit` - Maximální rychlosti pro poziční regulaci
- `accel_limit` - Maximální zrychlení poziční regulaci
- `decel_limit` - Maximální zpomalení poziční regulaci
- `traj_inertia`
- `input_pos` - Nastavení pozičního setpointu
- `velocity feed forward`
- `torque feed forward` 
- `position gain`

**Momentový regulační mód** 
- [ ] `input_torque` - Nastavení požadovaného momentu

**Proud na motoru**
- `comamnded motor current`
- `measured motor current`

**Teplota**
- *FET temperature*
- *motor temperature*

**Enkodery**
- *position estimate*
- *velocity estimate*

**Další nastavení**
- `ESTOP`
- `start anticogging`
- `reboot`
- `clear errors`
- `enter device firmware update mode`
- `set axis node ID` - ID, kterým se identifikuje na canu


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

Get_Encoder_Estimates  
- axis.encoder_.pos_estimate_
- axis.encoder_.vel_estimate_

Get_Iq  
- axis.motor_.current_control_.Idq_setpoint_
- axis.motor_.current_control_.Iq_measured_

Get_Sensorless_Estimates  
- axis.sensorless_estimator_.pll_pos_
- axis.sensorless_estimator_.vel_estimate_

Get_Vbus_Voltage  
- vbus_voltage

### Nastavovane parametry 
Set_Axis_Node_Id      
- axis.config_.can.node_id  

Set_Axis_State    
- axis.requested_state_  

Set_Controller_Mode  
- axis.controller_.config_.control_mode   
- axis.controller_.config_.input_mode   

Set_Input_Pos     
- axis.controller_.set_input_pos()
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



