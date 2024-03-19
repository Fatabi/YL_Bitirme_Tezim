%% Version Descriptions

%% feat: add navigation controllers
% - add lateral PD controller
% - add TECS controller for altitude following
% - integrate controllers with low lewel controllers

%% feat: add collision detection and basic benchmark manuever
% - zero wings level and 5G pull-up GCAS maneuver is added
% - collision detection module is integrated into simulink
% - GCAS switch handling added to autopilot
% - Visualization on matlab is integrated into Simulink Model

%% Fix UDP receive bugs.
% - Detect UDP delay is 1 timestep.
% - Detect initial UDP dead value length is 2~3 timestep
% - Update Unity exe to DTED version.

%% v1.3.0
% Date          : 19/11/2023 [DD/MM/YYYY]
% Creator       : Ege C. Altunkaya
% Modified by   : Fatih Erol
% Description   : - Change euler to quaternion in equations of motion
%  	              - Change signal logging frequency to inherited from simulation
%                 - Change trim condition initial North and East to 0

%% v1.2.0
% Date          : 16/11/2023 [DD/MM/YYYY]
% Creator       : Ege C. Altunkaya
% Modified by   : Fatih Erol
% Description   : - Add DCM's
%	              - Add Unity radar point receiver.
% 	              - Change signal logging frequency to 50 [Hz]

%% v1.1.0
% Date          : 12/11/2023 [DD/MM/YYYY]
% Creator       : Ege C. Altunkaya
% Modified by   : Fatih Erol
% Description   : - Fix propulsion tables
% 	              - Add joystick inputs
% 	              - Organize blocks

%% v0.0.2
% Date          : 03/11/2023 [DD/MM/YYYY]
% Creator       : Ege C. Altunkaya
% Modified by   : Abdulbaki Sanlan
% Description   : - Created Roll Angle Hold AutoPilot.
%                 - Added couple P and R command for improving 
%                   beta stability.

%% v0.0.1
% Date          : 12/10/2023 [DD/MM/YYYY]
% Creator       : Ege C. Altunkaya
% Modified by   : Fatih Erol
% Description   : - Created
%                 - Re-arranged blocks.
%                 - Added reset in all integrators.
%                 - Created Unity Visualizer block.
%                 - Created G command to pitch rate analytical block.

