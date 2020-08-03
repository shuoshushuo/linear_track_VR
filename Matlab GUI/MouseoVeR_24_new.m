function varargout = MouseoVeR(varargin)

% Jeremy Cohen (2010-2014)
% this software can communicate (via serial port, usb) with the Behavioral 
% Control System (BCS) "MouseoVeR_(ver).pde"

delete(instrfind) % clear any open ports
clc
global gui % setup a global variable for the gui

idn = 'MouseoVeR - not connected...';
flag_reset=0;

% Serial port communication/settings.
s=serial('COM9');
s.baudrate=115200;
s.flowcontrol='none';
s.inputbuffersize=600000;
s.bytesavailablefcnmode = 'terminator';
s.bytesavailablefcn=@receiveData;
set(s,'Terminator','CR/LF');
set(s,'DataBits',8);
set(s,'StopBits',2);
set(s,'DataTerminalReady','off');


%--------------------------------------------------------------------------
% Variables
%--------------------------------------------------------------------------

% folder_1 = 'C:\Users\cohenj\Documents\Jeremy_Notes\Behavioral Control\Matlab_MouseoVeR\';
% folder_2 = 'C:\Users\cohenj\Documents\Jeremy_Notes\Behavioral Control\Matlab_MouseoVeR\data_tmp';
% folder_save = 'C:\Users\cohenj\Documents\Jeremy_Data\MouseoVeR_behav_data';
% 
% % Set current directory to Results directory
% addpath(folder_1);
% addpath(folder_2);
% cd(folder_1);


folder_1='C:\Users\Shuo Chen\Documents\BehavioralData\';
folder_2='C:\Users\Shuo Chen\Documents\BehavioralData\data_temp';
folder_save='C:\Users\Shuo Chen\Documents\BehavioralData\data';

addpath(folder_1);
addpath(folder_2);
cd(folder_1);

sessionName = 'xzvr'; %default session name
vrEnvName_tmp = 'ENTER VR FILE'; % VR environment name (e.g. OvalTrack-16)

trackNames = {'square_track_1_v5', 'square_track_3_v6', 'oval-track-28_14', 'linear-track-11_20','L-track-14_15','fig8-track-3_4','teardrop-track-2_28','tunnel-track-1_13','+ Add a track'};

% Set current date
today_tmp=date;
today_tmp2=datevec(today_tmp);
today = ['xz' num2str(today_tmp2(1))];
if today_tmp2(2) < 10
    today = [today '0' num2str(today_tmp2(2))];
else
    today = [today num2str(today_tmp2(2))];
end
if today_tmp2(3) < 10
    today = [today '0' num2str(today_tmp2(3))];
else
    today = [today num2str(today_tmp2(3))];
end
hekaFileName = today; % heka/PatchMaster file name for neural data

sessionTime = 0; %time zero at start
sessionNotesTitle = 'Session Notes:';
sessionNotes = ' Enter a note, then press "return" to write note to file ';
fileName = '';
fileSize = 0; %filesize at t(0)
RD_score = 0; %RD_score count at time 0
lap_cnt = 0; %lap count at time 0
lapMarker = 0; % default lapMarker tag for when "p0" is crossed in VR
rewardCount = 0; %reward count at time 0
rewardCnt_last = 0;
spikeCount = 0;
spikeCnt_last = 0;
lickCount = 0;
lickCnt_last = 0;
lickCnt_loop = 0;
lickCnt_loop_2 = 0;
rollMax = 1;
minRD = 0;

servopos = 400000; %default move-to position
sendString = '';
BC_time='0'; % must be a string
vr_time = 0; % initial timestamp for session timer
vr_x = 0; % xcoord for online plotting
vr_y = 0; % ycoord for online plotting
vr_z = 0; % zcoord for online plotting
vr_speed=0;
avgSpeed_tmp=[0,0,0,0,0];
avgSpeed_arraySize = length(avgSpeed_tmp); % used for faster calculation in loop
avgSpeed='0'; % must be as string
avgSpeed_plotTag = 0;
speedPlot_loop=0;
speed_th_curr=0;
looptime = 0;
max_ML_looptime = 0;
max_BC_looptime = 0;
notesFile = 0; % tag for notes file 0 is no file, 1=file created.
% SERVOMOTOR DEVICE #
% servomotor device 0 = addresses all servomotors
% servomotor device 1 = lickport
% servomotor device 2 = ball clamp 1
% servomotor device 3 = ball clamp 2
% servomotor device 4 = ball clamp 3
% servomotor device 5 = eye cover

% gui properties for entire window
gui.fig = figure('tag','MouseoVeR','numbertitle','off','menubar','none','name',idn,'visible','off', ...
    'Units','normalized','position',[0.004 0.01 0.99 0.903]);
set(gui.fig,'CloseRequestFcn',@closefcn);
set(gui.fig,'Visible','on');
set(gui.fig,'KeyPressFcn',@keyPress);

% axes1 properties - postition plot
x_range = [0 100000];
y_range = [0 100000];
% x_range = [32000 52000];
% y_range = [65000 85000];

gui.axes1 = axes('xlim',x_range,'xtick',x_range,'xticklabel',x_range, ...
    'ylim',y_range,'ytick',y_range, 'yticklabel',y_range, ...
    'position',[0.08 0.08 0.4 0.6],'nextplot','add');

% axes2 properties - avg speed plot
gui.axes2 = axes('xlim',[0.5 1.5],'xtick',[],'xticklabel',[],'ylim',[0 50],'ytick',[0 10 20 30 40 50],'yticklabel',{'0';'10';'20';'30';'40';'50'},...
    'position',[0.52 0.17 0.05 0.19],...
    'Color',[0.9 0.9 0.9],'YGrid','On','nextplot','replacechildren');

% axes3 properties - speed_th (speed threshold) plot - overlayed on axes2
gui.axes3 = axes('xlim',[0.5 1.5],'xtick',[],'xticklabel',[],'ylim',[0 50],'ytick',[0 10 20 30 40 50],'yticklabel',{},...
    'position',[0.52 0.17 0.05 0.19],...
    'Color',[0.9 0.9 0.9],'YGrid','Off','Visible','off');

gui.fig = get(gui.axes1,'parent');


%--------------------------------------------------------------------------
% gui objects
%--------------------------------------------------------------------------

% SESSION INFORMATION
gui.sessionNameTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 0.8 0.8], 'fontsize',13,...
    'Position',[0.04 0.858 0.125 0.038],'String','Session Name:',...
    'Callback', @sessionUpdateButtonCallback); %callback doesn't do anything. just for title purposes

gui.sessionNameEdit = uicontrol('Parent', gui.fig, 'Style','edit','String',sessionName,'fontsize',11,'Units','normalized', ...
    'Position',[0.17 0.858 0.1 0.042], 'Callback', @sessionNameEditCallback);

gui.vrEnvNameTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 0.8 0.8], 'fontsize',13,...
    'Position',[0.020 0.808 0.15 0.038],'String','Starting VR Environment:',...
    'Callback', @vrEnvNameButtonCallback); %callback doesn't do anything. just for title purposes

gui.vrEnvNameEdit = uicontrol('Parent', gui.fig, 'Style','edit','String',vrEnvName_tmp,'fontsize',11,'Units','normalized', ...
    'Position',[0.17 0.808 0.1 0.042],'Visible','off','Callback', @vrEnvNameEditCallback);

gui.vrEnvNamePopup = uicontrol('Parent', gui.fig,'Style','popup',...
    'String',trackNames,'fontsize',11,...
    'Units','Normalized','Position',[0.17 0.795 0.1 0.053], 'Callback',@vrEnvNamePopupCallback);

gui.hekaFileNameTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 0.8 0.8], 'fontsize',13,...
    'Position',[0.04 0.758 0.13 0.038],'String','Date or heka File:',...
    'Callback', @hekaFileNameCallback); %callback doesn't do anything. just for title purposes

gui.hekaFileName = uicontrol('Parent', gui.fig, 'Style','edit','String',hekaFileName,'fontsize',11,'Units','normalized', ...
    'Position',[0.17 0.758 0.1 0.042], 'Callback', @hekaFileNameCallback);


% RECORDING CONTROL
gui.recordingcontrol = uicontrol('Parent', gui.fig,'Style','text','fontsize',14,'Units','normalized', ...
    'BackGroundColor','white', 'Position',[0.30 0.92 0.21 0.04],'String','Recording Control');

gui.record = uicontrol('Parent', gui.fig,'Style','togglebutton','Units','normalized','HandleVisibility','callback', ...
    'BackGroundColor','green', 'fontsize',11,...
    'Value',0,'Position',[0.30 0.85 0.1 0.05],'String','Recording OFF','Callback', @hRecordingButtonCallback);

gui.pause = uicontrol('Parent', gui.fig,'Style','togglebutton','Units','normalized','HandleVisibility','callback', ...
    'BackGroundColor','red', 'fontsize',11,...
    'Value',0,'Position',[0.41 0.85 0.1 0.05],'String','Pause OFF','Callback', @hPauseButtonCallback);

gui.checkbox1_vidrec = uicontrol('Parent', gui.fig,'Style','checkbox','Units','normalized','HandleVisibility','callback', ...
    'Value',0,'Position',[0.30 0.8 0.2 0.05],'String','check to turn Video OFF with Recording OFF',...
    'BackgroundColor',[0.8 0.8 0.8],'Callback', @checkbox1_vidrecCallback);

gui.checkbox3_RDSclear = uicontrol('Parent', gui.fig,'Style','checkbox','Units','normalized','HandleVisibility','callback', ...
    'Value',0,'Position',[0.62 0.675 0.15 0.05],'String','check to auto-fwrite ''RDS_clear''',...
    'BackgroundColor',[0.8 0.8 0.8],'Callback', @checkbox3_RDSclearCallback);

gui.serialPortReset = uicontrol('Parent', gui.fig,'Style','pushbutton','Units','normalized','HandleVisibility','callback', ...
    'BackGroundColor',[0.9 0.7 0.1], 'fontsize',11,'Value',0,'tooltipstring','Shortcut: Alt + s',...
    'Position',[0.3 0.76 0.21 0.04], 'String','Serial Port Reset','Callback', @serialPortResetButtonCallback);


% SERVOMOTOR CONTROL
gui.servocontrol = uicontrol('Parent', gui.fig,'Style','text','fontsize',14,'Units','normalized', ...
    'BackGroundColor','white', 'Position',[0.54 0.92 0.21 0.04],'String','ServoMotor Control');

gui.homeposition = uicontrol('Parent', gui.fig,'Style','pushbutton','Units','normalized','HandleVisibility','callback', ...
    'BackGroundColor',[1 0.6 0], 'fontsize',11,'tooltipstring','Shortcut: Alt + h',...
    'Value',0,'Position',[0.54 0.85 0.1 0.05],'String','Home Lickport','Callback', @homeButtonCallback);

gui.endposition = uicontrol('Parent', gui.fig,'Style','pushbutton','Units','normalized','HandleVisibility','callback', ...
    'BackGroundColor','yellow', 'fontsize',11,'tooltipstring','Shortcut: Alt + e',...
    'Value',0,'Position',[0.65 0.85 0.1 0.05],'String','End Lickport','Callback', @endButtonCallback);

gui.moveServo = uicontrol('Parent', gui.fig,'Style','pushbutton','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor','yellow', 'fontsize',11,'tooltipstring','Shortcut: Alt + m',...
    'Value',0,'Position',[0.54 0.79 0.15 0.05],'String','Move Lickport to Position:',...
    'Callback', @moveServoButtonCallback);

gui.servoposEdit = uicontrol('Parent', gui.fig, 'Style','edit','String',servopos,'fontsize',10,'Units','normalized', ...
    'Position',[0.70 0.79 0.05 0.05], 'Callback', @servoposEditCallback);

% Ball Clamp
gui.clamp = uicontrol('Parent', gui.fig,'Style','togglebutton','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor','yellow', 'fontsize',11,'tooltipstring','Shortcut: Alt + b',...
    'Value',0,'Position',[0.65 0.73 0.1 0.05],'String','Ball Clamp OFF',...
    'Callback', @clampButtonCallback);

% visor / eye cover
gui.visor = uicontrol('Parent', gui.fig,'Style','togglebutton','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor','yellow', 'fontsize',11,'tooltipstring','Shortcut: Alt + i',...
    'Value',0,'Position',[0.54 0.73 0.1 0.05],'String','Visor OFF',...
    'Callback', @visorButtonCallback);


% BCS COMMUNICATION
gui.BCcontrol = uicontrol('Parent', gui.fig,'Style','text','fontsize',14,'Units','normalized', ...
    'BackGroundColor','white', 'Position',[0.78 0.92 0.2 0.04],'String','BCS Communication');

gui.videoRec = uicontrol('Parent', gui.fig,'Style','togglebutton','Units','normalized','HandleVisibility','callback', ...
    'BackGroundColor','green', 'fontsize',11,'tooltipstring','Shortcut: Alt + v',...
    'Value',0,'Position',[0.78 0.85 0.2 0.05],'String','Video OFF','Callback', @videoRecButtonCallback);

gui.training = uicontrol('Parent', gui.fig,'Style','togglebutton','Units','normalized','HandleVisibility','callback', ...
    'BackGroundColor','yellow', 'fontsize',11,'tooltipstring','Shortcut: Alt + t',...
    'Value',0,'Position',[0.78 0.79 0.2 0.05],'String','Training OFF','Callback', @trainingButtonCallback);

% CLASB: closed-loop auditory stimulation box
gui.clasb = uicontrol('Parent', gui.fig,'Style','togglebutton','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor','yellow', 'fontsize',11,'tooltipstring','Shortcut: Alt + c',...
    'Value',0,'Position',[0.78 0.73 0.20 0.05],'String','C.L.A.S.B. OFF',...
    'Callback', @clasbButtonCallback);

gui.checkbox2_speed_th_tag = uicontrol('Parent', gui.fig,'Style','checkbox','Units','normalized','HandleVisibility','callback', ...
    'Value',0,'Position',[0.78 0.675 0.2 0.05],'String','Manually set speed_th (un-check to automate)',...
    'BackgroundColor',[0.8 0.8 0.8],'Callback', @checkbox2_speed_th_tagCallback);

gui.sendStringTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 0.79 0.8], 'fontsize',13,...
    'Position',[0.515 0.64 0.125 0.038],'String','Send String:',...
    'Callback', @sendStringTitleCallback); %callback doesn't do anything. just for title purposes

gui.sendStringEdit = uicontrol('Parent', gui.fig, 'Style','edit','String',sendString,'fontsize',11,'Units','normalized', ...
    'Position',[0.62 0.64 0.36 0.042], 'Callback', @sendStringEditCallback);

gui.clearfig = uicontrol('Parent', gui.fig,'Style','pushbutton','Units','normalized','HandleVisibility','callback', ...
    'BackGroundColor',[0.95 0.65 0.9], 'fontsize',11,'Value',0,'tooltipstring','Shortcut: Alt + f',...
    'Position',[0.08 0.69 0.08 0.05],'String','Clear Figure','Callback', @clearfigButtonCallback);

gui.rewardDelivery = uicontrol('Parent', gui.fig,'Style','pushbutton','Units','normalized','HandleVisibility','callback', ...
    'BackGroundColor',[0.0 0.7 0.9], 'fontsize',11,'tooltipstring','Shortcut: Alt + r',...
    'Value',0,'Position',[0.38 0.69 0.1 0.05],'String','Deliver Reward','Callback', @rewardDeliveryButtonCallback);

gui.setXYworldTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 0.8 0.8], 'fontsize',11,...
    'Position',[0.10 0.015 0.1 0.05],'String','Set X-Y for World:',...
    'Callback', @setXYworldTitleCallback); %callback doesn't do anything. just for title purposes

gui.world_reset = uicontrol('Parent', gui.fig,'Style','pushbutton','Units','normalized','HandleVisibility','callback', ...
    'BackGroundColor',[0.95 0.65 0.9], 'fontsize',11,...
    'Value',0,'Position',[0.025 0.13 0.045 0.035],'String','Reset','Callback', @world_resetCallback);

gui.world_A = uicontrol('Parent', gui.fig,'Style','pushbutton','Units','normalized','HandleVisibility','callback', ...
    'BackGroundColor',[0.18 0.4 0.6], 'fontsize',11,...
    'Value',0,'Position',[0.2 0.035 0.045 0.035],'String','Linear','Callback', @world_ACallback);

gui.world_B = uicontrol('Parent', gui.fig,'Style','pushbutton','Units','normalized','HandleVisibility','callback', ...
    'BackGroundColor',[0.4 0.6 0.2], 'fontsize',11,...
    'Value',0,'Position',[0.25 0.035 0.045 0.035],'String','L','Callback', @world_BCallback);

gui.world_C = uicontrol('Parent', gui.fig,'Style','pushbutton','Units','normalized','HandleVisibility','callback', ...
    'BackGroundColor',[0.6 0.2 0.4], 'fontsize',11,...
    'Value',0,'Position',[0.3 0.035 0.045 0.035],'String','Fig8','Callback', @world_CCallback);

gui.world_D = uicontrol('Parent', gui.fig,'Style','pushbutton','Units','normalized','HandleVisibility','callback', ...
    'BackGroundColor',[0.2 0.6 0.4], 'fontsize',11,...
    'Value',0,'Position',[0.35 0.035 0.045 0.035],'String','Oval','Callback', @world_DCallback);

gui.world_E = uicontrol('Parent', gui.fig,'Style','pushbutton','Units','normalized','HandleVisibility','callback', ...
    'BackGroundColor',[0.4 0.4 0.6], 'fontsize',11,...
    'Value',0,'Position',[0.4 0.035 0.05 0.035],'String','Teardrop','Callback', @world_ECallback);


% SESSION STATISTICS
gui.sessionInfo = uicontrol('Parent', gui.fig,'Style','text','fontsize',14,'Units','normalized', ...
    'BackGroundColor','white', 'Position',[0.04 0.92 0.23 0.04],'String','Session Information');

gui.sessionStatsTitle = uicontrol('Parent', gui.fig,'Style','text','fontsize',14,'Units','normalized', ...
    'BackGroundColor','white', 'Position',[0.5 0.57 0.48 0.04],'String','Session Statistics');

gui.fileNameTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 0.8 0.8], 'fontsize',10,...
    'Position',[0.5 0.52 0.06 0.033],'String','File Name:',...
    'Callback', @fileNameButtonCallback); %callback doesn't do anything. just for title purposes

gui.fileName = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.9 0.9 1], 'fontsize',11,...
    'Position',[0.56 0.525 0.42 0.033],'String',fileName,...
    'Callback', @fileNameUpdateButtonCallback); %callback doesn't do anything. just for title purposes

gui.fileNameError = uicontrol('Parent', gui.fig,'Style','edit','Units','normalized','HandleVisibility','callback', ...
    'ForegroundColor',[1 1 1],'BackGroundColor',[0 0 0], 'Visible','off', ...
    'fontsize',30,'Value',0,'Position',[0.1 0.6 0.8 0.07],...
    'String','ERROR: fileName already exists!','Callback', @fileNameErrorEditCallback);

gui.sessionTimeTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 0.8 0.8], 'fontsize',10,...
    'Position',[0.5 0.475 0.11 0.033],'String','Session Time (sec):',...
    'Callback', @sessionUpdateButtonCallback); %callback doesn't do anything. just for title purposes

buttonColor = [0.7 0.7 0.7];
gui.sessionTimeUpdate = uicontrol('Parent', gui.fig, 'Style','pushbutton','string', sessionTime,'fontsize',18,'Units','normalized', ...
    'Position',[0.5 0.42 0.11 0.06],'BackgroundColor',buttonColor,'Callback', @sessionTimeUpdateCallback);

gui.ML_looptimeTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 0.8 0.8], 'fontsize',10,...
    'Position',[0.74 0.475 0.12 0.033],'String','Max ML Looptime (ms):',...
    'Callback', @sessionUpdateButtonCallback); %callback doesn't do anything. just for title purposes

gui.ML_looptimeUpdate = uicontrol('Parent', gui.fig, 'Style','pushbutton','string', max_ML_looptime,'fontsize',18,'Units','normalized', ...
    'Position',[0.74 0.42 0.12 0.06],'BackgroundColor',buttonColor, 'Callback', @ML_looptimeUpdateCallback);

gui.BC_looptimeTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 0.8 0.8], 'fontsize',10,...
    'Position',[0.87 0.475 0.12 0.033],'String','Max BC Looptime (us):',...
    'Callback', @sessionUpdateButtonCallback); %callback doesn't do anything. just for title purposes

gui.BC_looptimeUpdate = uicontrol('Parent', gui.fig, 'Style','pushbutton','string', max_BC_looptime,'fontsize',18,'Units','normalized', ...
    'Position',[0.87 0.42 0.12 0.06],'BackgroundColor',buttonColor, 'Callback', @BC_looptimeUpdateCallback);

gui.fileSizeTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 0.8 0.8], 'fontsize',10,...
    'Position',[0.62 0.475 0.11 0.033],'String','File Size (kB):',...
    'Callback', @fileSizeTitleCallback); %callback doesn't do anything. just for title purposes

gui.fileSizeUpdate = uicontrol('Parent', gui.fig, 'Style','pushbutton','string', fileSize,'fontsize',18,'Units','normalized', ...
    'Position',[0.62 0.42 0.11 0.06],'BackgroundColor',buttonColor, 'Callback', @fileSizeUpdateCallback);

gui.minRD = uicontrol('Parent', gui.fig, 'Style','pushbutton','string', minRD,'fontsize',10,'Units','normalized', ...
    'Position',[0.8 0.30 0.06 0.04], 'Callback', @minRDCallback);

gui.minRDTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 .8 .8], 'fontsize',10,...
    'Position',[0.73 0.293 0.07 0.04],'String','Min RD/Lap:',...
    'Callback', @minRDTitleCallback); %callback doesn't do anything. just for title purposes

gui.rollMax = uicontrol('Parent', gui.fig, 'Style','pushbutton','string', rollMax,'fontsize',10,'Units','normalized', ...
    'Position',[0.68 0.36 0.05 0.04], 'Callback', @rollMaxCallback);

gui.rollMaxTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 .8 .8], 'fontsize',10,...
    'Position',[0.605 0.353 0.07 0.04],'String','Max RD_Rolls:',...
    'Callback', @rollMaxTitleCallback); %callback doesn't do anything. just for title purposes

gui.rewardCount = uicontrol('Parent', gui.fig, 'Style','pushbutton','string', rewardCount,'fontsize',10,'Units','normalized', ...
    'Position',[0.68 0.30 0.05 0.04], 'Callback', @rewardCountUpdateCallback);

gui.rewardCntTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 0.8 0.8], 'fontsize',10,...
    'Position',[0.62 0.298 0.06 0.035],'String','Rewards:',...
    'Callback', @rewardCntTitleCallback); %callback doesn't do anything. just for title purposes

gui.lap_cnt = uicontrol('Parent', gui.fig, 'Style','pushbutton','string', lap_cnt,'fontsize',10,'Units','normalized', ...
    'Position',[0.80 0.36 0.06 0.04], 'Callback', @lap_cntCallback);

gui.lap_cntTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 0.8 0.8], 'fontsize',10,...
    'Position',[0.748 0.358 0.04 0.035],'String','Laps:',...
    'Callback', @lap_cntTitleCallback); %callback doesn't do anything. just for title purposes

gui.RD_score = uicontrol('Parent', gui.fig, 'Style','pushbutton','string', [num2str(RD_score) ' %'],'fontsize',10,'Units','normalized', ...
    'Position',[0.80 0.24 0.06 0.04], 'Callback', @RD_scoreCallback);

gui.RD_scoreTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 0.8 0.8], 'fontsize',10,...
    'Position',[0.738 0.238 0.06 0.035],'String','Avg RD/lap:',...
    'Callback', @RD_scoreTitleCallback); %callback doesn't do anything. just for title purposes

gui.lickCount = uicontrol('Parent', gui.fig, 'Style','pushbutton','string', lickCount,'fontsize',10,'Units','normalized', ...
    'Position',[0.68 0.24 0.05 0.04], 'Callback', @lickCountUpdateCallback);

gui.lickCntTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 0.8 0.8], 'fontsize',10,...
    'Position',[0.62 0.246 0.06 0.025],'String','Licks:',...
    'Callback', @lickCntTitleCallback); %callback doesn't do anything. just for title purposes

gui.spikeCount = uicontrol('Parent', gui.fig, 'Style','pushbutton','string', spikeCount,'fontsize',10,'Units','normalized', ...
    'Position',[0.68 0.18 0.05 0.04], 'Callback', @spikeCountUpdateCallback);

gui.spikeCntTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 .8 .8], 'fontsize',10,...
    'Position',[0.62 .178 0.06 0.035],'String','Spikes:',...
    'Callback', @spikeCntTitleCallback); %callback doesn't do anything. just for title purposes

gui.avgSpeed = uicontrol('Parent', gui.fig, 'Style','togglebutton','string', avgSpeed,'fontsize',15,'Units','normalized', ...
    'Position',[0.513 0.115 0.065 0.04], 'Callback', @avgSpeedCallback);

gui.avgSpeedTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 .8 .8], 'fontsize',10,...
    'Position',[0.49 .37 0.1 0.025],'String','Speed (cm/s)',...
    'Callback', @avgSpeedTitleCallback); %callback doesn't do anything. just for title purpose

gui.sessionNotesTitle = uicontrol('Parent', gui.fig,'Style','text','Units','normalized','HandleVisibility', 'callback', ...
    'BackGroundColor',[0.8 .8 .8], 'fontsize',11,'HorizontalAlignment','left',...
    'Position',[0.5 .065 0.2 0.035],'String',sessionNotesTitle,...
    'Callback', @sessionNotesTitleCallback); %callback doesn't do anything. just for title purposes

gui.sessionNotesUpdate = uicontrol('Parent', gui.fig, 'Style','edit','String',sessionNotes,'FontSize',11,'Units','normalized', ...
    'Position',[0.5 0.025 0.45 0.05], 'HorizontalAlignment','left','Callback', @sessionNotesUpdateCallback);

% set rec tag to off
gui.recOn = false;

% setup avg speed figure
set(gui.avgSpeed,'BackgroundColor',buttonColor);
gui.speedBar=plot(gui.axes2,0,'*k','markerSize',10);
set(gui.axes2,'xlim',[0.5 1.5],'xtick',[],...
    'ylim',[0 50],'ytick',[0 10 20 30 40 50],...
    'yticklabel',{'0';'10';'20';'30';'40';'50'});

% set speed plot to common settings
gui.speed_thBar=plot(gui.axes3,[0.55:0.03:1.45],speed_th_curr,'*r','markerSize',2);
set(gui.axes3,'xlim',[0.5 1.5],'ylim',[0 50],'Visible','Off');
text(1.55,speed_th_curr,'threshold','fontSize',8,'Color','r');

% set VR environment name object to default option, value = 1
str_list = get(gui.vrEnvNamePopup,'String');
vrEnvName = str_list{1};

% set lickport position (servomotor 1) on startup
[entry,status] = str2num(get(gui.servoposEdit,'string'));
if status
    value = round(entry);
    if value >= 0 && value <= 550000
        set(gui.servoposEdit,'string', num2str(value));
        servopos = num2str(value);
    else
        set(gui.servoposEdit,'string', servopos);
    end
else
    set(gui.servoposEdit,'string', servopos);
end

% check the "manually set speed_th" checkbox_2
set(gui.checkbox2_speed_th_tag,'Value',1);

% check the "auto-write RDS_clear" checkbox_3
%maxVal = get(gui.checkbox3_RDSclear,'Max');
set(gui.checkbox3_RDSclear,'Value',1);

% open serial port
set(s,'DataTerminalReady','on');
fopen(s);
pause(0.1);

initialize =1; % tag to run initialization stuff in the first loop of receiveData


%----------------------------------------------------------------------
% CALLBACKS
%--------------------------------------------------------------------------

    function hRecordingButtonCallback(hObject, ~)
        button_state = get(hObject,'Value');
        if button_state == get(hObject,'Max')
            fileName = sprintf('%s_MouseoVeR_%s_%s.txt',sessionName,vrEnvName,hekaFileName);
            folder_list = dir(folder_1);
            for i=1:length(folder_list)
                if strcmp(folder_list(i).name,fileName)
                    set(gui.fileNameError,'Visible','On');
                    disp('ENTER NEW (sessionName_vrEnvironment_date) FILENAME IN TEXTBOX');
                    disp('OR');
                    disp('PRESS CTRL+C to CANCEL');
                    input('PRESS RETURN AFTER ENTERING NEW FILENAME');
                    fileName = get(gui.fileNameError,'String');
                    args = regexp(fileName, '\_','split');
                    animalName = args{1};
                    session = args{2};
                    sessionName = sprintf('%s_%s',args{1},args{2});
                    set(gui.sessionNameEdit, 'String', sessionName);
                    vrEnvName = args{3};
                    set(gui.vrEnvNameEdit,'String',vrEnvName);
                    hekaFileName_tmp = regexp(args{4},'\.','Split');
                    hekaFileName = hekaFileName_tmp{1};
                    set(gui.hekaFileName,'String',hekaFileName);
                    set(gui.fileNameError,'Visible','Off');
                end
            end
            folder_list = dir(folder_2);
            for i=1:length(folder_list)
                if strcmp(folder_list(i).name,fileName)
                    set(gui.fileNameError,'Visible','On');
                    disp('ENTER NEW (sessionName_vrEnvironment_date) FILENAME IN TEXTBOX');
                    disp('OR');
                    disp('PRESS CTRL+C to CANCEL');
                    input('PRESS RETURN AFTER ENTERING NEW FILENAME');
                    fileName = get(gui.fileNameError,'String');
                    args = regexp(fileName, '\_','split');
                    animalName = args{1};
                    session = args{2};
                    sessionName = sprintf('%s_%s',args{1},args{2});
                    set(gui.sessionNameEdit, 'String', sessionName);
                    vrEnvName = args{3};
                    set(gui.vrEnvNameEdit,'String',vrEnvName);
                    hekaFileName_tmp = regexp(args{4},'\.','Split');
                    hekaFileName = hekaFileName_tmp{1};
                    set(gui.hekaFileName,'String',hekaFileName);
                    set(gui.fileNameError,'Visible','Off');
                end
            end
            
            gui.recOn = true;
            set(hObject,'BackgroundColor','red');
            set(hObject,'String','Recording ON');
            gui.logfileID = fopen(fileName,'w');
            set(gui.fileName,'String',fileName);
            sessionNotesTitle = sprintf('session Notes: %s',sessionName);
            set(gui.sessionNotesTitle,'String',sessionNotesTitle);
            fileNotes = sprintf('%s_Notes.txt',sessionName);
            gui.logsessionNotes = fopen(fileNotes,'w');
            notesFile=1;
            fwrite(s,sprintf('videoRecON\r'));
            set(gui.videoRec,'String','Video ON');
            set(gui.videoRec,'BackgroundColor','red');
            fileSize = 0; %filesize at t(0)
            spikeCount = 0;
            spikeCnt_last = 0;
            lickCount = 0;
            lickCnt_last = 0;
            lickCnt_loop = 0;
            lickCnt_loop_2 = 0;
            set(gui.rewardCount,'String','0');
            rewardCount = 0;
            rewardCnt_last = 0;
            RD_score=0;
            set(gui.RD_score,'String','0 %');
            fwrite(s,sprintf('RD_score 0\r'));
            fwrite(s,sprintf('RD_cnt_curr 0\r'));
            fwrite(s,sprintf('RD_cnt 0\r'));
            fwrite(s,sprintf('lap_cnt 0\r'));
            lap_cnt = 0;
            set(gui.lap_cnt,'String','0');
            set(gui.lickCount,'String','0');
            spikeCount = 0;
            spikeCnt_last = 0;
            set(gui.spikeCount,'String','0');
            set(gui.fileSizeUpdate,'String','0');
            max_ML_looptime = 0;
            
        elseif button_state == get(hObject,'Min')
            gui.recOn = false;
            fclose(gui.logfileID);
            fclose(gui.logsessionNotes);
            set(hObject,'BackgroundColor','green');
            set(hObject,'String','Recording OFF');
            
            if (get(gui.checkbox1_vidrec,'Value') == get(gui.checkbox1_vidrec,'Max'))
                % Checkbox is checked-take approriate action
                % this stops the signal to acquire and save video frames.
                fwrite(s,sprintf('videoRecOFF\r'));
                set(gui.videoRec,'String','Video OFF');
                set(gui.videoRec,'BackgroundColor','green');
            else
                % Checkbox is NOT checked-take approriate action
                % keep video signal on.
            end
            
            fileNotes = sprintf('%s_Notes.txt',sessionName);
            file_path_notes = sprintf('%s%s',folder_1,fileNotes);
            copyfile(file_path_notes,folder_save);
            file_path_fileName = sprintf('%s%s',folder_1,fileName);
            copyfile(file_path_fileName,folder_save);
            notesFile=0;
        end
        
        set(gui.fileNameError,'Visible','Off');        
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function hPauseButtonCallback(hObject, ~)
        button_state = get(hObject,'Value');
        recButton_state = get(gui.record,'Value');
     
        if recButton_state == get(gui.record,'Max');
            if button_state == get(hObject,'Max')
                set(hObject,'BackgroundColor','green');
                set(hObject,'String','Pause ON');
                gui.recOn = false;
                fwrite(s,sprintf('videoRecOFF\r'));
                set(gui.videoRec,'String','Video OFF');
                set(gui.videoRec,'BackgroundColor','green');
            elseif button_state == get(hObject,'Min')
                set(hObject,'BackgroundColor','red');
                set(hObject,'String','Pause OFF');
                gui.recOn = true;
                fwrite(s,sprintf('videoRecON\r'));
                set(gui.videoRec,'String','Video ON');
                set(gui.videoRec,'BackgroundColor','red');
            end
        end
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function serialPortResetButtonCallback(hObject, ~)
        % reset serial port communication/settings.
        % can take some time, ~5-10 sec to reset.
        delete(instrfind) % clear any open ports
        s=serial('COM6');
        s.baudrate=115200;
        s.flowcontrol='none';
        s.inputbuffersize=500000;
        s.bytesavailablefcnmode = 'terminator';
        s.bytesavailablefcn=@receiveData;
        set(s,'Terminator','CR/LF');
        set(s,'DataBits',8);
        set(s,'StopBits',2);
        set(s,'DataTerminalReady','On');
        fopen(s);
        
        % auto-check the "speed_th checkbox"
        maxVal = get(gui.checkbox2_speed_th_tag,'Max');
        set(gui.checkbox2_speed_th_tag,'Value',maxVal);
        initialize =1; % tag to auto-set speed_th during first loop
        
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
        
        flag_reset=1;
        
    end

%----------------------------------------------------------------------

    function videoRecButtonCallback(hObject, ~)
        videobutton_state = get(hObject,'Value');
        if videobutton_state == get(hObject,'Max')
            fwrite(s,sprintf('videoRecON\r'));
            set(hObject,'String','Video ON');
            set(hObject,'BackgroundColor','red');
        elseif videobutton_state == get(hObject,'Min')
            fwrite(s,sprintf('videoRecOFF\r'));
            set(hObject,'String','Video OFF');
            set(hObject,'BackgroundColor','green');
        end
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function sessionNotesUpdateCallback(hObject,~)
        sessionNotes_temp = get(hObject,'String');
        sessionNotes = sprintf('t=%s %s',num2str(round(sessionTime)),sessionNotes_temp);
        if (gui.recOn == true)
            fprintf(gui.logsessionNotes,'%s\r\n',sessionNotes);
        end
        set(hObject,'String','');
        
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
       
        if notesFile
            fileSizeUpdateCallback(gui.fileSizeUpdate);
        end
    end

%----------------------------------------------------------------------

    function homeButtonCallback(hObject, ~)
        fwrite(s,sprintf('moveto 1 0 \r'));
        if (gui.recOn == true)
            sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'lickport moved to Home');
            fprintf(gui.logsessionNotes,'%s\r\n',sendString);
        end
        set(gui.homeposition,'BackgroundColor',[1 0.6 0]);
        set(gui.endposition,'BackgroundColor','y');
        set(gui.moveServo,'BackgroundColor','y');
        
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
        if notesFile
            fileSizeUpdateCallback(gui.fileSizeUpdate);
        end
    end

%----------------------------------------------------------------------

    function endButtonCallback(hObject, ~)
        fwrite(s,sprintf('moveto 1 400000 \r'));
        if (gui.recOn == true)
            sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'lickport moved to End');
            fprintf(gui.logsessionNotes,'%s\r\n',sendString);
        end
        set(gui.homeposition,'BackgroundColor','y');
        set(gui.endposition,'BackgroundColor',[1 0.6 0]);
        set(gui.moveServo,'BackgroundColor','y');
        
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
        if notesFile
            fileSizeUpdateCallback(gui.fileSizeUpdate);
        end
    end

%----------------------------------------------------------------------

    function moveServoButtonCallback(hObject, ~)
        fwrite(s,sprintf('moveto 1 %s\r',servopos));
        % 'moveto 1' addresses servomotor 1, the lickport
        if (gui.recOn == true)
            sendString = sprintf('t=%s %s %s',num2str(round(sessionTime)),'lickport moved to',servopos);
            fprintf(gui.logsessionNotes,'%s\r\n',sendString);
        end
        set(gui.homeposition,'BackgroundColor','y');
        set(gui.endposition,'BackgroundColor','y');
        set(gui.moveServo,'BackgroundColor',[1 0.6 0]);
        
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
        if notesFile
            fileSizeUpdateCallback(gui.fileSizeUpdate);
        end
    end

%----------------------------------------------------------------------

    function clampButtonCallback(hObject, ~,~)
        clampbutton_state = get(hObject,'Value');
        if clampbutton_state == get(hObject,'Max')
            fwrite(s,sprintf('movet0 0 240000\r'));
            fwrite(s,sprintf('moveto 3 220000\r'));
            fwrite(s,sprintf('moveto 4 260000\r'));
            set(hObject,'String','Ball Clamp ON');
            set(hObject,'BackgroundColor',[1 0.6 0]);
            if (gui.recOn == true)
                sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'Ball Clamp On');
                fprintf(gui.logsessionNotes,'%s\r\n',sendString);
            end
        elseif clampbutton_state == get(hObject,'min')
            fwrite(s,sprintf('moveto 2 1000\r'));
            fwrite(s,sprintf('moveto 3 1000\r'));
            fwrite(s,sprintf('moveto 4 1000\r'));
            set(hObject,'String','Ball Clamp OFF');
            set(hObject,'BackgroundColor','yellow');
            if (gui.recOn == true)
                sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'Ball Clamp Off');
                fprintf(gui.logsessionNotes,'%s\r\n',sendString);
            end
        end
        
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
        if notesFile
            fileSizeUpdateCallback(gui.fileSizeUpdate);
        end
    end

%----------------------------------------------------------------------

    function visorButtonCallback(hObject, ~,~)
        % Visor servomotor properties
        visorEnd = 10000;
        visorStart = visorEnd - 4500;
        fwrite(s,sprintf('setMaxPos 5 %s \r',num2str(visorEnd)));
        visorOnSpeed = 50;
        visorOffSpeed = 30;
        
        visorbutton_state = get(hObject,'Value');
        if visorbutton_state == get(hObject,'Max')
            fwrite(s,sprintf('setSpeed 5 %s \r',num2str(visorOnSpeed)));
            fwrite(s,sprintf('moveto 5 %s \r',num2str(visorEnd)));
            set(hObject,'String','Visor ON');
            set(hObject,'BackgroundColor',[1 0.6 0]);
            if (gui.recOn == true)
                sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'Visor On');
                fprintf(gui.logsessionNotes,'%s\r\n',sendString);
            end
        elseif visorbutton_state == get(hObject,'min')
            fwrite(s,sprintf('setSpeed 5 %s \r',num2str(visorOffSpeed)));
            fwrite(s,sprintf('moveto 5 %s \r',num2str(visorStart)));            set(hObject,'String','Visor OFF');
            set(hObject,'BackgroundColor','yellow');
            if (gui.recOn == true)
                sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'Visor Off');
                fprintf(gui.logsessionNotes,'%s\r\n',sendString);
            end
        end
        
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
        if notesFile
            fileSizeUpdateCallback(gui.fileSizeUpdate);
        end
    end

%----------------------------------------------------------------------


    function clasbButtonCallback(hObject,~)
        % CLASB: closed-loop auditory stimulation box
        clasbbutton_state = get(hObject,'Value');
        if clasbbutton_state == get(hObject,'Max')
            fwrite(s,sprintf('ASS_A 1\r'));
            set(hObject,'String','C.L.A.S.B. ON');
            set(hObject,'BackgroundColor',[1 0.6 0]);
            set(hObject,'Value',1);
            if (gui.recOn == true)
                sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'ASS_A On');
                fprintf(gui.logsessionNotes,'%s\r\n',sendString);
            end
        elseif clasbbutton_state == get(hObject,'min')
            fwrite(s,sprintf('ASS_A 0\r'));
            set(hObject,'String','C.L.A.S.B. OFF');
            set(hObject,'BackgroundColor','yellow');
            set(hObject,'Value',0);
            if (gui.recOn == true)
                sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'ASS_A Off');
                fprintf(gui.logsessionNotes,'%s\r\n',sendString);
            end
        end
        
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
        if notesFile
            fileSizeUpdateCallback(gui.fileSizeUpdate);
        end
    end

%----------------------------------------------------------------------

    function checkbox1_vidrecCallback(hObject, ~)
        box_state = get(hObject,'Value');
        % basically do nothing... used in other function to check state
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function checkbox2_speed_th_tagCallback(hObject, ~)
        box_state = get(hObject,'Value');
        if box_state == get(hObject,'Max')
            % Checkbox is checked-take approriate action
            fwrite(s,sprintf('speed_th_tag 0\r'));
        else
            % Checkbox is NOT checked-take approriate action
            fwrite(s,sprintf('speed_th_tag 1\r'));
        end
        
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function checkbox3_RDSclearCallback(hObject, ~, ~)
        box_state = get(hObject,'Value');
        % basically do nothing... used in other function to check state
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function servoposEditCallback(hObject, ~, ~)
        [entry,status] = str2num(get(hObject,'string'));
        if (status)
            value = round(entry);
            if (value >= 0 && value <= 550000)
                set(hObject,'string', num2str(value));
                servopos = num2str(value);
            else
                set(Object,'string', servopos);
            end
        else
            set(hObject,'string', servopos);
        end
        
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function sessionNameEditCallback(hObject, ~, ~)
        sessionName = get(hObject,'string');
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function vrEnvNameEditCallback(hObject, ~, ~)
        vrEnvName = get(hObject,'String');
        trackNames(length(trackNames)+1) = trackNames(length(trackNames));
        trackNames(length(trackNames)-1) = cellstr(vrEnvName);
        set(gui.vrEnvNamePopup,'String',trackNames);
        set(gui.vrEnvNamePopup,'Value',length(trackNames)-1);
        set(gui.vrEnvNameEdit,'Visible','Off');
        set(gui.vrEnvNamePopup,'Visible','On');

        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function vrEnvNamePopupCallback(hObject, ~, ~)
        val = get(hObject,'Value');
        str_list = get(hObject,'String');
        vrEnvName = str_list{val};
        if strcmp(vrEnvName,'+ Add a track')
            set(gui.vrEnvNamePopup,'Visible','Off');
            set(gui.vrEnvNameEdit,'Visible','On');
        end
        
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function hekaFileNameCallback(hObject, ~, ~)
        hekaFileName = get(hObject,'string');
        
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function fileNameErrorEditCallback(hObject, ~, ~)
        fileName = get(hObject,'string');
        
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function lickCountUpdateCallback(hObject, ~)
        button_state = get(hObject, 'Value');
        if button_state == get(hObject, 'Max')
            lickCount = 0;
            lickCnt_last = 0;
            set(hObject,'String', lickCount);
        elseif button_state == get(hObject, 'Min')
            lickCount = 0;
            lickCnt_last = 0;
            set(hObject,'String', lickCount);
        end
        
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function sessionTimeUpdateCallback(hObject, ~)
        % reset the clock display to 0
        sessionTime = '0';
        set(hObject,'String', sessionTime);
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function fileSizeUpdateCallback(hObject, ~)
        if (gui.recOn == true)
            file_path = sprintf('%s%s',folder_1,fileName);
            file_info = dir(file_path);
            fileSize_bytes = file_info.bytes;
            fileSize_kb = round(fileSize_bytes/1024);
            fileSize = num2str(fileSize_kb);
            set(gui.fileSizeUpdate,'String',fileSize);
        else
            fileSize = 0;
            set(gui.fileSizeUpdate,'String',fileSize);
        end
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function spikeCountUpdateCallback(hObject, ~)
        button_state = get(hObject, 'Value');
        if button_state == get(hObject, 'Max')
            spikeCount = 0;
            spikeCnt_last = 0;
            set(hObject,'String', spikeCount);
        elseif button_state == get(hObject, 'Min')
            spikeCount = 0;
            spikeCnt_last = 0;
            set(hObject,'String', spikeCount);
        end
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function rewardCountUpdateCallback(hObject, ~)
        button_state = get(hObject, 'Value');
        if button_state == get(hObject, 'Max')
            rewardCount = 0;
            rewardCnt_last = 0;
            set(hObject,'String', rewardCount);
        elseif button_state == get(hObject, 'Min')
            rewardCount = 0;
            rewardCnt_last = 0;
            set(hObject,'String', rewardCount);
        end
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function avgSpeedCallback(hObject, ~)
        button_state = get(hObject, 'Value');
        if button_state == get(hObject, 'Max')
            avgSpeed_plotTag = 1;
            speedPlot_loop = 0;
            set(hObject,'BackgroundColor',[0.3 0.3 1]);
        elseif button_state == get(hObject, 'Min')
            avgSpeed_plotTag = 0;
            speedPlot_loop =0;
            set(hObject,'BackgroundColor',[0.5 0.5 0.6]);
            gui.speedBar=plot(gui.axes2,0,'*k','markerSize',9);
        end
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function trainingButtonCallback(hObject, ~)
        button_state = get(hObject,'Value');
        if button_state == get(hObject,'Max')
            fwrite(s,sprintf('startTraining\r'));
            set(hObject,'BackgroundColor','magenta');
            set(hObject,'String','Training On');
            if (gui.recOn == true)
                sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'Training On');
                fprintf(gui.logsessionNotes,'%s\r\n',sendString);
            end
            
        elseif button_state == get(hObject,'Min')
            fwrite(s,sprintf('stopTraining\r'));
            set(hObject,'BackgroundColor','yellow');
            set(hObject,'String','Training Off');
            if (gui.recOn == true)
                sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'Training Off');
                fprintf(gui.logsessionNotes,'%s\r\n',sendString);
            end
        end
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function clearfigButtonCallback(hObject, ~)
        cla(gui.axes1);

        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
        if notesFile
            fileSizeUpdateCallback(gui.fileSizeUpdate);
        end

    end


%----------------------------------------------------------------------

    function world_resetCallback(hObject, ~)
        % axes1 properties - postition plot
        x_range = [1000 30000];
        y_range = [1000 30000];
        set(gui.axes1,'xlim',x_range,'xtick',x_range,'xticklabel',x_range, ...
            'ylim',y_range,'ytick',y_range, 'yticklabel',y_range)
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function world_ACallback(hObject, ~)
        % axes1 properties - postition plot
        x_range = [10000 29000];
        y_range = [10000 29000];
        set(gui.axes1,'xlim',x_range,'xtick',x_range,'xticklabel',x_range, ...
            'ylim',y_range,'ytick',y_range, 'yticklabel',y_range)
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function world_BCallback(hObject, ~)
        % axes1 properties - postition plot
        x_range = [8000 27000];
        y_range = [5000 24000];
        set(gui.axes1,'xlim',x_range,'xtick',x_range,'xticklabel',x_range, ...
            'ylim',y_range,'ytick',y_range, 'yticklabel',y_range)
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function world_CCallback(hObject, ~)
        % axes1 properties - postition plot
        x_range = [16000 29000];
        y_range = [15000 27000];
        set(gui.axes1,'xlim',x_range,'xtick',x_range,'xticklabel',x_range, ...
            'ylim',y_range,'ytick',y_range, 'yticklabel',y_range)
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function world_DCallback(hObject, ~)
        % axes1 properties - postition plot
        x_range = [2000 9000];
        y_range = [1000 11000];
        set(gui.axes1,'xlim',x_range,'xtick',x_range,'xticklabel',x_range, ...
            'ylim',y_range,'ytick',y_range, 'yticklabel',y_range)
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function world_ECallback(hObject, ~)
        % axes1 properties - postition plot
        x_range = [22200 31200];
        y_range = [35000 44000];
        set(gui.axes1,'xlim',x_range,'xtick',x_range,'xticklabel',x_range, ...
            'ylim',y_range,'ytick',y_range, 'yticklabel',y_range)
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function rewardDeliveryButtonCallback(hObject, ~)
        fwrite(s,sprintf('%s\r','reward'));
        fileSizeUpdateCallback(gui.fileSizeUpdate);
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function ML_looptimeUpdateCallback(hObject, ~)
        button_state = get(hObject, 'Value');
        if button_state == get(hObject, 'Max')
            max_ML_looptime = 0;
            set(gui.ML_looptimeUpdate,'String',0);
        elseif button_state == get(hObject, 'Min')
            max_ML_looptime = 0;
            set(gui.ML_looptimeUpdate,'String',0);
        end
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function BC_looptimeUpdateCallback(hObject, ~)
        button_state = get(hObject, 'Value');
        if button_state == get(hObject, 'Max')
            max_BC_looptime = 0;
            fwrite(s,sprintf('loopTimeMaxReset\r'));
        elseif button_state == get(hObject, 'Min')
            max_BC_looptime = 0;
            fwrite(s,sprintf('loopTimeMaxReset\r'));
        end
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
    end

%----------------------------------------------------------------------

    function sendStringEditCallback(hObject,~)
        % get the input string
        sendString_temp = get(hObject,'String');
        sendString = sprintf('%s',sendString_temp);
        
        % if its an "RDS" command...
        if numel(sendString) > 3 % safety check that string is long enough to be RDS
            if strcmp(sendString(1:3),'RDS')
                % check if auto-write RDSclear checkbox is checked...
                button_state = get(gui.checkbox3_RDSclear,'Value');
                if button_state == get(gui.checkbox3_RDSclear,'Max')
                    fwrite(s,sprintf('%s\r','RDS_clear'));
                end
            end
        end
        
        % fwrite string typed into the edit box
        fwrite(s,sprintf('%s\r',sendString));
        
        % if rec is ON, fprintf string in the edit box to notes file
        if (gui.recOn == true)
            sendString2 = sprintf('t=%s %s',num2str(round(sessionTime)),sendString);
            fprintf(gui.logsessionNotes,'%s\r\n',sendString2);
        end
        
        % parse the input line
        sendString_parse = regexp(sendString,'\ ','Split');
        if strcmp(sendString_parse(1),'RD_cnt');
            rewardCount = str2num(sendString_parse{2});
            rewardCnt_last = rewardCount;
            set(gui.rewardCount, 'String', num2str(rewardCount))
        elseif strcmp(sendString_parse(1),'clear');
            cla(gui.axes1);
        elseif strcmp(sendString_parse(1),'rollMax');
            rollMax = str2num(sendString_parse{2});
            set(gui.rollMax,'String',num2str(rollMax));
        elseif strcmp(sendString_parse(1),'lap_cnt');
            lap_cnt = str2num(sendString_parse{2});
            set(gui.lap_cnt,'String',num2str(lap_cnt));
        elseif strcmp(sendString_parse(1),'RD_score_reset');
            lap_cnt =0;
            set(gui.lap_cnt,'String','0');
            RD_score=0;
            set(gui.RD_score,'String','0 %');
        elseif strcmp(sendString_parse(1),'RDS_A4') && strcmp(sendString_parse(2),'1')
            fwrite(s,sprintf('rollMax 1\r'));
            set(gui.rollMax,'String','1');
            world_ACallback(gui.world_A);
        elseif strcmp(sendString_parse(1),'RDS_B4') && strcmp(sendString_parse(2),'1')
            fwrite(s,sprintf('rollMax 1\r'));
            set(gui.rollMax,'String','1');
            world_BCallback(gui.world_B);
        elseif strcmp(sendString_parse(1),'RDS_C4') && strcmp(sendString_parse(2),'1')
            fwrite(s,sprintf('rollMax 1\r'));
            set(gui.rollMax,'String','1');
            world_CCallback(gui.world_C);
        elseif strcmp(sendString_parse(1),'RDS_D4') && strcmp(sendString_parse(2),'1')
            fwrite(s,sprintf('rollMax 1\r'));
            set(gui.rollMax,'String','1');
            world_DCallback(gui.world_D);
        elseif strcmp(sendString_parse(1),'RDS_E4') && strcmp(sendString_parse(2),'1')
            fwrite(s,sprintf('rollMax 1\r'));
            set(gui.rollMax,'String','1');
            world_ECallback(gui.world_E);
        end
        
        % clear edit box
        set(hObject,'String','');
        set(hObject, 'Enable', 'off'); drawnow; set(hObject, 'Enable', 'on');
        if notesFile
            fileSizeUpdateCallback(gui.fileSizeUpdate);
        end

    end

%----------------------------------------------------------------------

    function keyPress(hObject,event)
        if strcmp(event.Modifier,'alt') & strcmp(event.Character,'s')
            % reset serial port communication/settings.
            % can take some time, ~up to 5 sec to reset.
            serialPortResetButtonCallback(gui.serialPortReset);
        elseif strcmp(event.Modifier,'alt') & strcmp(event.Character,'v')
            % toggle the video button state to turn ON/OFF video signal
            videobutton_state = get(gui.videoRec,'Value');
            if videobutton_state < 1
                fwrite(s,sprintf('videoRecON\r'));
                set(gui.videoRec,'String','Video ON');
                set(gui.videoRec,'BackgroundColor','red');
                set(gui.videoRec,'Value',1);
            elseif videobutton_state
                fwrite(s,sprintf('videoRecOFF\r'));
                set(gui.videoRec,'String','Video OFF');
                set(gui.videoRec,'BackgroundColor','green');
                set(gui.videoRec,'Value',0);
            end
        elseif strcmp(event.Modifier,'alt') & strcmp(event.Character,'c')
            % toggle the CLASB: closed-loop auditory stimulation box
            clasbbutton_state = get(gui.clasb,'Value');
            if clasbbutton_state < 1
                fwrite(s,sprintf('ASS_A 1\r'));
                set(gui.clasb,'String','C.L.A.S.B. ON');
                set(gui.clasb,'BackgroundColor',[1 0.6 0]);
                set(gui.clasb,'Value',1)
                if (gui.recOn == true)
                    sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'ASS_A On');
                    fprintf(gui.logsessionNotes,'%s\r\n',sendString);
                end
            elseif clasbbutton_state
                fwrite(s,sprintf('ASS_A 0\r'));
                set(gui.clasb,'String','C.L.A.S.B. OFF');
                set(gui.clasb,'BackgroundColor','yellow');
                set(gui.clasb,'Value',0);
                if (gui.recOn == true)
                    sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'ASS_A Off');
                    fprintf(gui.logsessionNotes,'%s\r\n',sendString);
                end
            end
        elseif strcmp(event.Modifier,'alt') & strcmp(event.Character,'t')
            trainingbutton_state = get(gui.training,'Value');
            if trainingbutton_state < 1
                fwrite(s,sprintf('startTraining\r'));
                set(gui.training,'BackgroundColor','magenta');
                set(gui.training,'String','Training On');
                set(gui.training,'Value',1)
                if (gui.recOn == true)
                    sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'Training On');
                    fprintf(gui.logsessionNotes,'%s\r\n',sendString);
                end
            elseif trainingbutton_state
                fwrite(s,sprintf('stopTraining\r'));
                set(gui.training,'BackgroundColor','yellow');
                set(gui.training,'String','Training Off');
                set(gui.training,'Value',0)
                if (gui.recOn == true)
                    sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'Training Off');
                    fprintf(gui.logsessionNotes,'%s\r\n',sendString);
                end
            end
        elseif strcmp(event.Modifier,'alt') & strcmp(event.Character,'i')
            % toggle the Visor button
            % Visor servomotor properties
            visorEnd = 10000;
            visorStart = visorEnd - 4500;
            fwrite(s,sprintf('setMaxPos 5 %s \r',num2str(visorEnd)));
            visorOnSpeed = 50;
            visorOffSpeed = 30;
            
            visorbutton_state = get(gui.visor,'Value');
            if visorbutton_state < 1
                fwrite(s,sprintf('setSpeed 5 %s \r',num2str(visorOnSpeed)));
                fwrite(s,sprintf('moveto 5 %s \r',num2str(visorEnd)));
                set(gui.visor,'String','Visor ON');
                set(gui.visor,'BackgroundColor',[1 0.6 0]);
                set(gui.visor,'Value',1)
                if (gui.recOn == true)
                    sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'Visor On');
                    fprintf(gui.logsessionNotes,'%s\r\n',sendString);
                end
            elseif visorbutton_state
                fwrite(s,sprintf('setSpeed 5 %s \r',num2str(visorOffSpeed)));
                fwrite(s,sprintf('moveto 5 %s \r',num2str(visorStart)));
                set(gui.visor,'String','Visor OFF');
                set(gui.visor,'BackgroundColor','yellow');
                set(gui.visor,'Value',0)
                if (gui.recOn == true)
                    sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'Visor Off');
                    fprintf(gui.logsessionNotes,'%s\r\n',sendString);
                end
            end
        elseif strcmp(event.Modifier,'alt') & strcmp(event.Character,'b')
            clampbutton_state = get(gui.clamp,'Value');
            if clampbutton_state < 1
                fwrite(s,sprintf('movet0 0 240000\r'));
                fwrite(s,sprintf('moveto 3 220000\r'));
                fwrite(s,sprintf('moveto 4 260000\r'));
                set(gui.clamp,'String','Ball Clamp ON');
                set(gui.clamp,'BackgroundColor',[1 0.6 0]);
                set(gui.clamp,'Value',1);
                if (gui.recOn == true)
                    sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'Ball Clamp On');
                    fprintf(gui.logsessionNotes,'%s\r\n',sendString);
                end
            elseif clampbutton_state
                fwrite(s,sprintf('moveto 2 1000\r'));
                fwrite(s,sprintf('moveto 3 1000\r'));
                fwrite(s,sprintf('moveto 4 1000\r'));
                set(gui.clamp,'String','Ball Clamp OFF');
                set(gui.clamp,'BackgroundColor','yellow');
                set(gui.clamp,'Value',0);
                if (gui.recOn == true)
                    sendString = sprintf('t=%s %s',num2str(round(sessionTime)),'Ball Clamp Off');
                    fprintf(gui.logsessionNotes,'%s\r\n',sendString);
                end
            end
        elseif strcmp(event.Modifier,'alt') & strcmp(event.Character,'m')    
            % move the lickport to location set in text edit box
            moveServoButtonCallback(gui.moveServo);
        elseif strcmp(event.Modifier,'alt') & strcmp(event.Character,'h')
            % move lickport to Home position
            homeButtonCallback(gui.homeposition);
        elseif strcmp(event.Modifier,'alt') & strcmp(event.Character,'e')
            % move lickport to End position
            endButtonCallback(gui.endposition);
        elseif strcmp(event.Modifier,'alt') & strcmp(event.Character,'f')
            % clear figure/position plot
            clearfigButtonCallback(gui.clearfig);
        elseif strcmp(event.Modifier,'alt') & strcmp(event.Character,'r')
            % deliver Reward
            rewardDeliveryButtonCallback(gui.rewardDelivery);
        elseif strcmp(event.Modifier,'alt') & strcmp(event.Character,'q')
            % close the main window
            closefcn(gui.fig);
        end
        
        if notesFile
            fileSizeUpdateCallback(gui.fileSizeUpdate);
        end
        
    end

%----------------------------------------------------------------------

%--------------------------------------------------------------------------
% VARIABLE DESCRIPTIONS FOR receiveData CALLBACK
%--------------------------------------------------------------------------
%
% BC_tstamp = Behavioral control box timestamp (microsec clock) printed at
%    microsec resolution
% Bevt = Behavioral event type (e.g. VR/RD/TH/LK/FF/PM)
% VR_tstamp = timestamp from VR (mouseoVeR) computer
% xcoord/ycoord/zcoord = animal x/y/z/ position in VR
% speed = animal average speed during previous time window/frame
% dir = animal average head-direction

%----------------------------------------------------------------------
% receive incoming data stream from serial port (s), a long string
%inpline % test
    function receiveData(~,~,~)
        tic;
        inpline = fgetl(s); % get data from serial object
        
        if (gui.recOn == true)
            fprintf(gui.logfileID,'%s\r\n',inpline); % write data to file,
        end
        
        % might not be necessary but clearing java heap memory can help with
        % repetitive plotting issues if they arise.
        heapTotalMemory = java.lang.Runtime.getRuntime.totalMemory;
        heapFreeMemory = java.lang.Runtime.getRuntime.freeMemory;
        if (heapFreeMemory < (heapTotalMemory*0.01))
            java.lang.Runtime.getRuntime.gc;
        end
        
        args = regexp(inpline, '\,','split');
        %  BC_time = args{1}; % unused
        
        args                  
        
        mm=numel(args) ;      
        if mm>=2        
            Bevt = args(2)  % 2-letter descriptor of behavioral event
            if strcmp(Bevt,'FR')&&(flag_reset==1)
                msgbox('Reset Done!');
                flag_reset=0;
            end
        else
            Bevt='NULL';
        end
                
        
        flag_err=0;
        if ~isstrprop(char(args(1)),'digit')
            flag_err=1;
            errmsg='args(1) error'
        end
        if ~iscellstr(args(2))
            flag_err=1;
            errmsg='args(2) error'
        end
        if numel(args)>=3
            if numel(args(3))>=2
                flag_err=1;
            end
            if strcmp('VR',Bevt)
                for temp_loop=3:(min(numel(args),10))
                    if ~isstrprop(char(args(temp_loop)),'digit')
                        flag_err=1;
                        errmsg=strcat('args(',num2str(temp_loop),') error')
                    end
                end
            end
        end
        if flag_err==1
            beep; beep; beep;
        end
        
        if strcmp('VR', Bevt) % data line from VR system (MouseoVeR)
            if (numel(args) < 12)&&(flag_err==0)
                [BC_tstamp,Bevt,vr_time,vr_x,vr_y,vr_z,vr_speed,vr_dir,eventCnt,eventstr_1,eventstr_2,eventstr_3,eventstr_4] = strread(inpline,'%u%s%u%u%u%u%u%d%u%s%s%s%s','delimiter',',');
                
                if isempty(vr_speed)
                    vr_speed = 0;
                end
                
                %  average speed vector
                if vr_time > 5000 % 5 sec wait time
                    for i = 2:avgSpeed_arraySize
                        avgSpeed_tmp(i-1) = avgSpeed_tmp(i); % shift values 1 row earlier
                    end
                    avgSpeed_tmp(avgSpeed_arraySize) = vr_speed; % add current value to last row
                    avgSpeed = mean(avgSpeed_tmp)/100; % average the vector
                    %                 avgSpeed = vr_speed/100; % for plotting instantaneous value
                    set(gui.avgSpeed,'String',num2str(round(avgSpeed)));
                    speedPlot_loop=speedPlot_loop+1;
                    if (speedPlot_loop==4) && (avgSpeed_plotTag) % only plot every 5th value
                        gui.speedBar=plot(gui.axes2,avgSpeed,'*b','markerSize',9);
                        set(gui.axes2,'xlim',[0.5 1.5],'xtick',[],'ylim',[0 50],'ytick',[0 10 20 30 40 50],'yticklabel',{'0';'10';'20';'30';'40';'50'});
                        speedPlot_loop=0;
                    end
                end
            end
            
        elseif strcmp('LK', Bevt) % Lick event
            lickCount = (lickCnt_last + 1);
            set(gui.lickCount, 'String', lickCount);
            lickCnt_loop = lickCnt_loop + 1;
            lickCnt_last = (lickCnt_last + 1);
            lickCnt_loop_2 = lickCnt_loop_2 + 1;
            
            if (lickCnt_loop == 3) % plot every 4th lick
                gui.dot=plot(gui.axes1,vr_x, vr_y,'bX','markerSize',4);
                lickCnt_loop = 0;
            end
            if (lickCnt_loop_2 > 300) % clear fig every 300 licks
                cla(gui.axes1);
                lickCnt_loop_2 = 0;
            end
            
        elseif strcmp('RD', Bevt) % Reward Delivery event
            rewardCount = rewardCnt_last + 1;
            set(gui.rewardCount, 'String', num2str(rewardCount));
            gui.dot=plot(gui.axes1,vr_x, vr_y,'go','markerSize',7);
            rewardCnt_last = (rewardCnt_last) + 1;
            
        elseif strcmp('TH', Bevt) % Vm Threshold crossing event (spike) detected
            spikeCount = (spikeCnt_last + 1);
            set(gui.spikeCount, 'String', spikeCount);
            gui.dot=plot(gui.axes1,vr_x, vr_y,'r.');
            spikeCnt_last = (spikeCnt_last + 1);
            
            % CURRENTLY UN-USED STRINGS FROM MOUSEOVER
            %         elseif (strcmp('FF', Bevt) % Frame Flash from projectors (image display marker)
            %             [BC_tstamp,Bevt] = strread(inpline,'%u%s','delimiter',',');
            %
            %         elseif strcmp('PM', Bevt) % Protocol Marker (from heka/Patchmaster)
            %             [BC_tstamp,Bevt] = strread(inpline,'%u%s','delimiter',',');
            %
            %         elseif (strcmp('Display_Blanking_On',Bevt) == true) % Display is Off (blacked out, world was previously visible)
            %             [BC_tstamp,Bevt] = strread(inpline,'%u%s','delimiter',',');
            %
            %         elseif (strcmp('Display_Blanking_Off',Bevt) == true) % Display is On (world is visible, was previously blacked out)
            %             [BC_tstamp,Bevt] = strread(inpline,'%u%s','delimiter',',');
            %
            %         elseif (strcmp('Enable_Motion_On',Bevt) == true) % Motion enabled (ball controls VR)
            %             [BC_tstamp,Bevt] = strread(inpline,'%u%s','delimiter',',');
            %
            %         elseif (strcmp('Enable_Motion_Off',Bevt) == true) % Motion Disbled, (ball disconnected from VR)
            %             [BC_tstamp,Bevt] = strread(inpline,'%u%s','delimiter',',');
            %
            
        elseif strcmp('MaxLoopTime',Bevt) % MaxLoopTime for chipKIT/Arduino Code (expected to be <300 usec)
            [BC_tstamp,Bevt,MLT] = strread(inpline,'%u%s%u','delimiter',',');
            max_BC_looptime = MLT;
            set(gui.BC_looptimeUpdate,'String',max_BC_looptime);
            
        elseif strcmp('speed_th',Bevt) % speed threshold
            [BC_tstamp,Bevt,speed_th] = strread(inpline,'%u%s%u','delimiter',',');
            speed_th_curr = speed_th/100;
            cla(gui.axes3);
            gui.speed_thBar=plot(gui.axes3,[0.55:0.04:1.48],speed_th_curr,'*r','markerSize',2);
            set(gui.axes3,'xlim',[0.5 1.5],'ylim',[0 50],'Visible','Off'); hold on;
            axes(gui.axes3);
            text(1.55,speed_th_curr,'threshold','fontSize',8,'Color','r');
            
        elseif strcmp('lapMarker',Bevt) % lapMarker tag
            [BC_tstamp,Bevt,lapMarker] = strread(inpline,'%u%s%u','delimiter',',');
            %       disp(['lapMarker = p' num2str(lapMarker)]);
            if lapMarker == 0;
                lap_cnt = lap_cnt + 1;
                lap_cnt_str = num2str(lap_cnt);
                set(gui.lap_cnt,'String',lap_cnt_str);
            end
            
        elseif strcmp('RD_score',Bevt) % avg num of RDs received per lap (RDs/lap)
            [BC_tstamp,Bevt,RD_score] = strread(inpline,'%u%s%u','delimiter',',');
            %            disp(['RD_score = ' num2str(RD_score)]);
            RD_score_str = [num2str(RD_score*0.01*minRD)];
            set(gui.RD_score,'String',RD_score_str);
            
        elseif strcmp('minRD',Bevt) % min num of RDs delivered per lap in the RD zone
            [BC_tstamp,Bevt,minRD] = strread(inpline,'%u%s%u','delimiter',',');
            %            disp(['minRD = ' num2str(minRD)]);
            minRD_str = num2str(minRD);
            set(gui.minRD,'String',minRD_str);
            
        elseif strcmp('rollMax',Bevt) % max num of roll attempts per randRDroll sequence
            [BC_tstamp,Bevt,rollMax] = strread(inpline,'%u%s%u','delimiter',',');
            %            disp(['rollMax = ' num2str(rollMax)]);
            rollMax_str = num2str(rollMax);
            set(gui.rollMax,'String',rollMax_str);
            
        elseif strcmp('FR', Bevt) % Firmware Revision, update Fig name when loading up
            [BC_tstamp, Bevt, name] = strread(inpline,'%u%s%s','delimiter',',');
            set(gui.fig,'name',char(name));
            
        end
        
        looptime = toc; % check receiveData function looptime
        if (looptime > max_ML_looptime) % if greater than current max, update
            max_ML_looptime_curr_tmp = looptime * 1e3;
            max_ML_looptime_curr = sprintf('%.1f',max_ML_looptime_curr_tmp);
            set(gui.ML_looptimeUpdate,'String',max_ML_looptime_curr);
            max_ML_looptime = looptime;
        elseif (looptime > 0.02) % if greater than 20ms, update
            max_ML_looptime_curr_tmp = looptime * 1e3;
            max_ML_looptime_curr = sprintf('%.1f',max_ML_looptime_curr_tmp);
            set(gui.ML_looptimeUpdate,'String',max_ML_looptime_curr);
            max_ML_looptime = looptime;
        end
        
        sessionTime = (vr_time/1000);
        set(gui.sessionTimeUpdate, 'String', num2str(round(sessionTime)));
        
        if initialize % only executes this stuff once on startup
            fwrite(s,sprintf('speed_th_tag 0\r')) % turn off tag for auto setting speed_th
            fwrite(s,sprintf('speed_th 4000\r')) % auto-set speed_th to a high value
            fwrite(s,sprintf('moveto 1 0 \r')); % send the lickport to Home position
            fwrite(s,sprintf('videoRecON\r')); % turn on video ON signal
            set(gui.videoRec,'String','Video ON','BackgroundColor','red'); % set button display to reflect state
            initialize=0; 
        end
        
    end

%----------------------------------------------------------------------

    function closefcn(hObject,~)
        delete(s);
        delete(hObject);
        fclose('all');
    end

%----------------------------------------------------------------------

end



