clear all
close all
clc

        %no of vehicles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             n=10;                
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch(n)
    case 1 
        scenarioPath = [getenv('SUMO_HOME') '\doc\tutorial\platoon\data1\platoon.sumocfg'];
    case 2
        scenarioPath = [getenv('SUMO_HOME') '\doc\tutorial\platoon\data2\platoon.sumocfg'];
    case 3
        scenarioPath = [getenv('SUMO_HOME') '\doc\tutorial\platoon\data3\platoon.sumocfg'];
    case 4
        scenarioPath = [getenv('SUMO_HOME') '\doc\tutorial\platoon\data4\platoon.sumocfg'];
    case 5
        scenarioPath = [getenv('SUMO_HOME') '\doc\tutorial\platoon\data5\platoon.sumocfg'];
    case 6  
        scenarioPath = [getenv('SUMO_HOME') '\doc\tutorial\platoon\data6\platoon.sumocfg'];
    case 7
        scenarioPath = [getenv('SUMO_HOME') '\doc\tutorial\platoon\data7\platoon.sumocfg'];
    case 8
        scenarioPath = [getenv('SUMO_HOME') '\doc\tutorial\platoon\data8\platoon.sumocfg'];
    case 9
        scenarioPath = [getenv('SUMO_HOME') '\doc\tutorial\platoon\data9\platoon.sumocfg'];
    case 10
        scenarioPath = [getenv('SUMO_HOME') '\doc\tutorial\platoon\data10\platoon.sumocfg'];
    otherwise
        n=4;
        scenarioPath = [getenv('SUMO_HOME') '\doc\tutorial\platoon\data4\platoon.sumocfg'];
end


try
	system(['sumo-gui -c ' scenarioPath '&']);
catch err
end

sumoHome = getenv('SUMO_HOME');
if isempty(strfind(sumoHome,'-'))
    sumoVersion = 'unknown';
    testVehicle = 'right_10';
else
    sumoVersion = str2double(sumoHome(9:12));
    if sumoVersion < 0.20
        testVehicle = '20';
    else
        testVehicle = 'right_20';
    end
end
%update myTable with packet reception details
myTable = matfile('table1.mat');
myTable.Properties.Writable = true;
receptionTable = zeros(n,n);

subscribedToTestVeh = 0;
contextSubsToTestVeh = 0;

import traci.constants

% Initialize TraCI
traci.init();

%% GUI SET COMMANDS
traci.gui.setSchema('View #0',  'real world');


step = 0;

current_pos={};
current_speed_veh={};
C=zeros(1,n*2);
A=zeros(1,n);
B=zeros(1,n*5);
X=zeros(1,n);
V=zeros(1,n*2);

Vx=0;
Vy=0;



% Creating a TCP/IP connection with NS-3 i.e present in different operating
% system
t1 = tcpip('192.168.56.101', 8000, 'NetworkRole', 'client'); 
set(t1, 'InputBufferSize', 600000); 
%set(t1,'Timeout',1);

fopen(t1);  % opening the connection  


while traci.simulation.getMinExpectedNumber()>0
    % Here, we demonstrate how to use the simulationStep command using an
    % argument. 
 traci.simulationStep(100*step);

%    pause(0.1);
   tic
%     traci.simulationStep();

% SHOW THE VEHICLES IDS INSIDE THE NETWORK
     vehicles = traci.vehicle.getIDList();
    
%    fprintf('IDs of the vehicles in the simulation\n')
          for j=1:length(vehicles)
                veh{j} = vehicles{j};

                current_pos{j}=traci.vehicle.getPosition(veh{j});
                current_speed_veh{j} = traci.vehicle.getSpeed(veh{j});
                current_angle{j} = traci.vehicle.getAngle(veh{j});
                   
                   
                C=cell2mat(current_pos);
                A=cell2mat(current_speed_veh);
                X=cell2mat(current_angle);

              vehicleNumber = sprintf('veh%d', n-1);
 if (length(vehicles)==n)    
  if(veh{j}==vehicleNumber) 
    
    % Here calucalting velocity vector from speed magnitude and angle. THis is very simple vector theory  
   for i=1:n
          Vx=A(i)*cosd(X(i))  ;
          Vy=A(i)*sind(X(i));
                    if(Vx==0)
        
                         Vx = Vy;
                         Vy=0;
         
                     elseif (Vy==0)
            
                        Vy=Vx;
                         Vx=0;
        
                    end
                    Y=[Vx Vy];
                        if (i==1)
                         V=[Vx Vy];
                     
                        else
                        V=horzcat(V,Y);
                    end
      
      end
         
       Vf=horzcat(V,X);
       
     
   
    B=horzcat(C,Vf);
               
               time=0.1*step;
                 
               B;

%  For receiving data from NS-3.

flushinput(t1);
flt='%5.4f,';
if (step==0)%send total number of vehicles in B[1]
    B = horzcat(n,B);
    fltAll=repmat(flt,1,n*5+1);
    fprintf(t1,fltAll(1:end),B);
    clear B;B=zeros(1,n*5);
else
    fltAll=repmat(flt,1,n*5);
    fprintf(t1,fltAll(1:end),B);
end      
    pause(5);
    act_data=zeros(1,n*3*(n-1));
    while (get(t1, 'BytesAvailable') > 0) 
        DataReceived = fread(t1,t1.BytesAvailable);
        act_data = DataReceived';   %% Transposing the column vector 'DataReceived'
    end
    [row,col]=size(act_data);
    receptionTable = 100*ones(n,n);%set the table entry to 100 every step. It denotes delay in milliseconds
    myTable.nd_1 = receptionTable;% If entry>99, no reception or reception is discarded. Rows->receiver & colomns->senders
    
    for i = 1:col
        if (mod(i,3)==1)
            receptionTable(str2num(char(act_data(i)))+1,(str2num(char(act_data(i+1)))+1))=str2num(char(act_data(i+2)));
        end
    end
    myTable.nd_1 = receptionTable;
   flushinput(t1);

   
%    Controlling SUMO based on the data received from NS3
for i = 1:n
   traci.vehicle.setSpeedMode(sprintf('veh%d', i-1),0);
   traci.vehicle.setSpeed(sprintf('veh%d', i-1),22);
end
%{
if (step>10)
    traci.vehicle.setSpeed('veh5',32);
end
%}
                toc
                end
            end

        end
 step = step + 1;

end
           
traci.close()
