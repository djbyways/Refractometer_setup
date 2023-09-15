function varargout = Refractometer_v01(varargin)
% REFRACTOMETER_V01 MATLAB code for Refractometer_v01.fig
%      REFRACTOMETER_V01, by itself, creates a new REFRACTOMETER_V01 or raises the existing
%      singleton*.
% 
%      H = RE FRACTOMETER_V01 returns the handle to a new REFRACTO METER_V01 or the handle to
%      the existing singleton*.
%
%      REFRACTOMETER_V01('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in REFRACT OMETER_V01.M with the given input arguments.
%
%      REFRACTOMETER_V01('Property','Value',...) creates a new REFRACTOMETER_V01 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Refractometer_v01_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Refractometer_v01_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Refractometer_v01

% Last Modified by GUIDE v2.5 08-Jun-2022 09:39:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Refractometer_v01_OpeningFcn, ...
                   'gui_OutputFcn',  @Refractometer_v01_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT
% 
    

% --- Executes just before Refractometer_v01 is made visible.
function Refractometer_v01_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Refractometer_v01 (see VARARGIN)

% Choose default command line output for Refractometer_v01
handles.output = hObject;
global H
disp('Create handles to displayed objects..');
    % Create handles to plotyy: hayy are handles to axees, plts - to
    % plotted objects
    H.h_ax = handles.axes1; % axes handles passed to global
    [H.hayy,H.h_plt1,H.h_plt2] = plotyy(H.h_ax,0,0,0,0); 
    hold on;
    H.h_Vline = plot([0,0],[0,0],'b'); % blue line initialised
    hold off;
    grid on;
    xlabel('Pixels')
    ylabel('Intensity [arb.u.]')
%     H.h_ax = handles.axes1; % axes handles passed to global
    set( handles.ui_table,'ColumnName',{'Screw','Raw_Ind','Lambda','Ref_Ind','Temperature'});
    H.data = [];
disp('Camera initialisation..'); 
try
%     handles.cam = webcam('HP Webcam');
      handles.cam = gigecam;
      handles.cam.PixelFormat = 'Mono16';
      handles.cam.AcquisitionFrameRateAbs = 20;
      handles.cam.AcquisitionFrameRate = 20;
      disp('Mono16 pixel format, FrameRate = 20 fps'); 
      handles.cam.ExposureTime = 30000;
      handles.cam.GevSCPSPacketSize = 4000;
%       fps = CalculateFrameRate(g, 20);
      delay = CalculatePacketDelay(handles.cam, 20);
      handles.cam.GevSCPD = delay;
      handles.cam.Timeout = 100;
      set(handles.ed_Exposition,'string',num2str(handles.cam.ExposureTime));
      

catch
    % handles.cam = webcam('USB2.0 Camera');
    display('Problems with camera initialisation');
end
%% 
disp('Thermometer initialisation..');
    handles.rs232  = instrfind('Type', 'serial', 'Port', 'COM1', 'Tag', '');
    handles.fid=[];
    if isempty(handles.rs232)
        handles.rs232 = serial('COM1', 'BaudRate', 1200, 'Parity', 'EVEN', 'StopBits', 1, 'DataBits', 7);
    else
        fclose(handles.rs232);
        handles.rs232 = handles.rs232(1);
    end
    fopen(handles.rs232);
    fprintf(handles.rs232,'A'); % Activate data transmission
    d = fscanf(handles.rs232);  % Read data from device, Some times it returns junks
    
    while length(d)~=32
        d = fscanf(handles.rs232);  % Read port until recive proper data vector
    end
%     if isequal() 
%         
%     end
        
   Tc = 'KJTERSN';  % Aveilabel type of termocouples
   id = strfind(Tc,d(8)); % read the number of current termocouple
    for i = 1:8-id  % set the proper one. In our case it is 'K'
        fprintf(handles.rs232,'F');
        fprintf(handles.rs232,'L');
        pause(0.25)
    end
    % The plot settings
    handles.T = zeros(20e3,4);
    handles.hf_Themperature = figure();
    handles.ha_Themperature = axes();
    handles.plt = plot(handles.ha_Themperature,...
        handles.T(:,1),handles.T(:,2),...
        handles.T(:,1),handles.T(:,3));
    grid on;
    xlabel( 'time[ min ]')
    ylabel('T ^oC')
    ylim([-15,60])
    legend('T1','T2')

    handles.rb_Cool_Heat.Value = 0;
    handles.rb_Cool_Heat.BackgroundColor = [1,0,0];

% Update handles structure
guidata(hObject, handles);
%% ==== User functions == 
    function xk = f_Kalman(z, K)
        N = length(z);
        xk = zeros(1,N);
        xk(1) = z(1);
        for t = 1:(N-1)
            xk(t+1) = xk(t)*(1-K) + K*z(t+1);
        end   


    function [time,T1,T2,TermocoupleType] = readT(d)
        % The function reads the data from CHY 506R device
        try
            T1 = hex2dec(d(2:7))/1e3;
            T2 =  hex2dec(d(10:15))/1e3;
            h = d(17:18);
            m = d(19:20);
            s = d(21:22);
            time = str2double(h)*3600 + str2double(m)*60 + str2double(s);
            TermocoupleType = [d(8),d(16)];
            if d(1)=='-'
                T1 = -1*T1;
            end
            if d(9)=='-'
                T2 = -1*T2;
            end

        catch
            T1 = NaN;
            T2 = NaN;
            time = NaN;
            TermocoupleType = NaN;
        end
% UIWAIT makes Refractometer_v01 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Refractometer_v01_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pb_Priview.
function pb_Priview_Callback(hObject, eventdata, handles)
% hObject    handle to pb_Priview (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global H 
hp = preview(handles.cam);
if (isfield(H, 'X') && get(handles.rb_blue_line,'Value')) 
   hold(hp.Parent, 'on'); 
   plot(hp.Parent,[H.X(2),H.X(2)],[H.Y(1)-30,H.Y(3)+30],'b');
end
 %% plotting a circle
% 
% %% circle on preview
% r = 165;    y = 267;    x = 189;    th = 0:pi/50:2*pi;
% xunit = r * cos(th) + x;    yunit = r * sin(th) + y;
% plot(hp.Parent,xunit, yunit,'r');
% %% rectangle on preview (use the same 'colx' and 'rowy' for roipoly)
% colx = [72 72 305 305 72];
% rowy = [153 386 386 153 153];
% % plot(hp.Parent,[72+(305-72)/2,72+(305-72)/2],[240,386],'b');
% plot(hp.Parent,colx,rowy,'r');




function ed_Screw_Callback(hObject, eventdata, handles)
% hObject    handle to ed_Screw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_Screw as text
%        str2double(get(hObject,'String')) returns contents of ed_Screw as a double

% --- Executes during object creation, after setting all properties.
function ed_Screw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_Screw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function ed_Raw_ref_ind_Callback(hObject, eventdata, handles)
% hObject    handle to ed_Raw_ref_ind (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global H
raw_ref = str2num( hObject.String );
Sq = str2num( get(handles.ed_Screw,'String') );

[lambda, n] = Ref_Converter(Sq,raw_ref);

% d = fscanf(handles.rs232);  % Read data from device, Some times it returns junks
%     while length(d)~=32
%         d = fscanf(handles.rs232);  % Read port until recive proper data vector
%     end
% [time,T1,T2,TermocoupleType] = readT(d); % FIXME: ustawiæ odpowiedni¹ termopare
H.data(end+1,:) = [Sq,raw_ref,lambda,n, H.T2]; %increase table dimension by one row.
handles.ui_table.Data = H.data;
set(handles.ed_Screw,'String','');
set(handles.ed_Raw_ref_ind,'String','');
% Hints: get(hObject,'String') returns contents of ed_Raw_ref_ind as text
%        str2double(get(hObject,'String')) returns contents of ed_Raw_ref_ind as a double
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function ed_Raw_ref_ind_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_Raw_ref_ind (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pb_Start.
function pb_Start_Callback(hObject, eventdata, handles)
% hObject    handle to pb_Start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global H

H.Run = true;
Frame1 = snapshot(handles.cam);
Frame_size = size(Frame1);
% Frame1 = Frame1(140:410,80:370);

choice = questdlg('Do you want to select the ROI?; Select 3pts on the top-left - bottom-right diagonal crosswire (Top, middle & bottom)', ...
   'Attention!!!', ...
	'Yes','No - previous values are OK','Yes'); 
if strcmp(choice,'Yes')	
    F = figure;
    imshow(Frame1,'Border','tight', 'InitialMagnification','fit');
    [H.X, H.Y] = ginput_dj(3); % red crosshairs, gap=1
    H.X = round(H.X);
    H.Y = round(H.Y);
    close(F)
    set(handles.ed_blue_line_pos,'String',num2str(H.X(2)));
else
   if ~isfield(H, 'X')
       warndlg('No ROI has been selected - select ROI', 'Warning');
       return;
   end
end
% --> Initialisation of the relay!!
% !CommandApp_USBRelay.exe 
kmax = 2; % averaging window
i = 1;
H.T2 = 0;
while H.Run
avrage = zeros(Frame_size,'double');
    for k = 1:kmax
       Frame = double(snapshot(handles.cam));
%           Frame = snapshot(handles.cam);
%           avrage = avrage + double(Frame);
        avrage = avrage + Frame;
        
         d = fscanf(handles.rs232);
           if length(d) == 32
               [time,T1,T2,TermocoupleType] = readT(d);
               handles.T(i,1) = time;
               handles.T(i,2) = T1;
               handles.T(i,3) = T2;
               H.T2 = T2;
               set(handles.plt(1),'xdata',handles.T(1:i,1)/60,'ydata',handles.T(1:i,2));
               set(handles.plt(2),'xdata',handles.T(1:i,1)/60,'ydata',handles.T(1:i,3));
               i = i + 1;
               if i >= 20e3
                   i = 1;
               end
           end
        
    end
  avrage = avrage./kmax;
  Frame_w_o_bg = avrage - H.average; % subtract rather than divide  
%% take region of interest specified by the vectors 'colx' and 'rowy'
    colx = [H.X(1) H.X(1) H.X(3) H.X(3)];
    rowy = [H.Y(1) H.Y(3) H.Y(3) H.Y(1)];
    roi = roipoly(Frame_w_o_bg,colx,rowy);
%% multiply RoI mask by the averaged image 
    edges = Frame_w_o_bg .* roi;
    edges = sum(edges(:,H.X(1):H.X(3)),1);
    edges = smooth(edges,0.19,'loess');
%% find derivative and smooth
    df = smooth(diff(edges),'sgolay',2); 
%     Y = sum(Frame,2);
    X1 = 1:length(edges); % x positions for edges
    X2 = 1:length(X1)-1; % x position for df
    % draw vertical blue line; due to ROI the position is relative to X(1)
    Vline_rel_X = H.X(2)-H.X(1);
%    set(H.h_Vline,'xdata',[Vline_rel_X-4 Vline_rel_X-4],'ydata',[min(edges),max(edges)]);
    set(H.h_Vline,'xdata',[Vline_rel_X-2.5 Vline_rel_X-2.5],'ydata',[min(edges),max(edges)]);   
   set(handles.ed_df_X2,'String',num2str(int32(df(Vline_rel_X))));
   
   set(H.h_plt1,'xdata',X1,'ydata',edges);
   H.hayy(1).XLim = [X1(1) X1(end)];
   H.hayy(1).YLim = [min(edges) max(edges)];
   set(H.h_plt2,'xdata',X2,'ydata',df);
   H.hayy(2).XLim = [X2(1) X2(end)];
   H.hayy(2).YLim = [1.05*min(df) 0.8*max(df)];
    drawnow;
  %% intensity value
%   xcol = [72 72 72 304 304];
%   yrow = [153 153 386 386 153];
%   Brite = roipoly(Frame,xcol,yrow);
%   Region = Frame.*double(Brite);
%   intens = mean2(Region(153:386,72:304));
% intens = int32(mean2(Frame(:,a:H.X(2))));

% --> we want to avoid saturation in ROI of original frame
maxintens = uint16(max(max(avrage(:,H.X(1):uint16(H.X(2)))))); 
%   set(handles.ed_intes,'string',[num2str(maxintens) '  ' num2str(intens)]);
if maxintens <= 65531
    set(handles.ed_intes,'ForegroundColor','g');
else
    set(handles.ed_intes,'ForegroundColor','r');
end
set(handles.ed_intes,'string',int2str(maxintens));
%% Temperature stabilisation section
%    d = fscanf(handles.rs232);
%            if length(d) == 32
%                [time,T1,T2,TermocoupleType] = readT(d);
%                handles.T(i,1) = time;
%                handles.T(i,2) = T1;
%                handles.T(i,3) = T2;
%                set(handles.plt(1),'xdata',handles.T(1:i,1)/60,'ydata',handles.T(1:i,2));
%                set(handles.plt(2),'xdata',handles.T(1:i,1)/60,'ydata',handles.T(1:i,3));
%                i = i + 1;
%                if i >= 20e3
%                    i = 1;
%                end
%            end
   T_ref = str2double(get(handles.ed_Temperature,'String'));       
%    switch handles.rb_Cool_Heat.Value
%        case 1   %% cooling
%            if T1 >= T_ref
%                !CommandApp_USBRelay ¿»MBS open 5;
%            else
%                !CommandApp_USBRelay ¿»MBS close 5;
%            end
%            
%        case 0
%            if T1 <= T_ref
%                !CommandApp_USBRelay ¿»MBS open 5;
%            else
%                !CommandApp_USBRelay ¿»MBS close 5;
%            end
%    end
                    
   
   
   
end
% !CommandApp_USBRelay ¿»MBS close 255
guidata(hObject, handles);


% --- Executes on button press in pb_Stop.
function pb_Stop_Callback(hObject, eventdata, handles)
% hObject    handle to pb_Stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global H
H.Run = false;
 
% --- Executes on button press in pb_Save.
function pb_Save_Callback(hObject, eventdata, handles)
% hObject    handle to pb_Save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fn = get(handles.ed_FileName,'String');
global H
if exist(fn)
    choice = questdlg('File alredy exist!, Do you want to Overwrite it?', ...
	'Warning!!!', ...
	'Yes','No','No');
    if strcmp(choice,'Yes')
        out = H.data;
        save(fn,'out','-ascii', '-double');
    end
else
    out = H.data;
    save(fn,'out','-ascii', '-double');
end

function ed_FileName_Callback(hObject, eventdata, handles)
% hObject    handle to ed_FileName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_FileName as text
%        str2double(get(hObject,'String')) returns contents of ed_FileName as a double


% --- Executes during object creation, after setting all properties.
function ed_FileName_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_FileName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pb_ClearData.
function pb_ClearData_Callback(hObject, eventdata, handles)
% hObject    handle to pb_ClearData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global H 
 choice = questdlg('Do you want to Clear the data?','Warning','Yes','No','No'); 
 if strcmp(choice,'Yes')
     H.data = [];
     handles.ui_table.Data = H.data;
 end
     

% --- Executes on button press in pb_get_BG.
function pb_get_BG_Callback(hObject, eventdata, handles)
% hObject    handle to pb_get_BG (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% collecting the dark background image
% !! different ed_exposition/gain requires a new backround !!
global H

choice = questdlg('Is the Field of View dark ??', ...
   'Attention!!!', ...
	'Yes','No','No'); 
if strcmp(choice,'No')	
       warndlg('Make the Field of View dark press the button again', 'Warning');
       return;
end
    Frame = snapshot( handles.cam );
    average = zeros(size(Frame),'double');
    Number_of_Frames = 100;
        wb = waitbar(0,'Colecting frames for average background...');
        for k = 1:Number_of_Frames
            waitbar(k/Number_of_Frames,wb);
            Frame = snapshot( handles.cam );
            average = average + double(Frame);
        end
    close(wb)    
    H.average = double(average) ./ Number_of_Frames;
  set(handles.pb_Start, 'Enable', 'on') ;

function ed_Temperature_Callback(hObject, eventdata, handles)
% hObject    handle to ed_Temperature (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_Temperature as text
%        str2double(get(hObject,'String')) returns contents of ed_Temperature as a double


% --- Executes during object creation, after setting all properties.
function ed_Temperature_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_Temperature (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in rb_Cool_Heat.
% function rb_Cool_Heat_Callback(hObject, eventdata, handles)
% % hObject    handle to rb_Cool_Heat (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % Hint: get(hObject,'Value') returns toggle state of rb_Cool_Heat
% if hObject.Value == 0
%     hObject.BackgroundColor = [1,0,0];
%     !CommandApp_USBRelay ¿»MBS close 5
%     
%     pause(1);
%     !CommandApp_USBRelay ¿»MBS open 6
% else
%      hObject.BackgroundColor = [.5,.5,1];
%      !CommandApp_USBRelay ¿»MBS close 5
%      
%      pause(1);
%      !CommandApp_USBRelay ¿»MBS close 6
% end


% --- Executes on button press in pb_clear_lastRow.
function pb_clear_lastRow_Callback(hObject, eventdata, handles)
% hObject    handle to pb_clear_lastRow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global H 
 choice = questdlg('Do you want to Clear the last row?','Warning','Yes','No','No'); 
 if strcmp(choice,'Yes')
     H.data(end,:) = []; %decrease table dimension by one row.
     handles.ui_table.Data(end,:) = [];
 end



function ed_Exposition_Callback(hObject, eventdata, handles)
% hObject    handle to ed_Exposition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_Exposition as text
%        str2double(get(hObject,'String')) returns contents of ed_Exposition as a double
exposure = str2double(get(hObject,'String'));
handles.cam.ExposureTime = exposure;
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function ed_Exposition_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_Exposition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_df_X2_Callback(hObject, eventdata, handles)
% hObject    handle to ed_df_X2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_df_X2 as text
%        str2double(get(hObject,'String')) returns contents of ed_df_X2 as a double

% derivative value at blue line (X2)

% --- Executes during object creation, after setting all properties.
function ed_df_X2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_df_X2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function ed_blue_line_pos_Callback(hObject, eventdata, handles)
% hObject    handle to ed_blue_line_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_blue_line_pos as text
%        str2double(get(hObject,'String')) returns contents of ed_blue_line_pos as a double
global H
if isfield(H,'X')
   H.X(2)= str2double(get(hObject,'String'));
end
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function ed_blue_line_pos_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_blue_line_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in rb_blue_line.
function rb_blue_line_Callback(hObject, eventdata, handles)
% hObject    handle to rb_blue_line (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rb_blue_line


% --- Executes on button press in pbResetCam.
function pbResetCam_Callback(hObject, eventdata, handles)
% hObject    handle to pbResetCam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if exist('handles.cam.ExposureTime','var')
    temp_exposure = handles.cam.ExposureTime;
else
    temp_exposure = 30000;
end
delete(handles.cam);
clear handles.cam
disp('Camera reinitialisation..'); 
try
      handles.cam = gigecam;
      handles.cam.PixelFormat = 'Mono16';
      handles.cam.AcquisitionFrameRateAbs = 20;
      handles.cam.AcquisitionFrameRate = 20;
      disp('Mono16 pixel format, FrameRate = 20 fps'); 
      handles.cam.ExposureTime = 30000;
      handles.cam.GevSCPSPacketSize = 4000;
%       fps = CalculateFrameRate(g, 20);
      delay = CalculatePacketDelay(handles.cam, 20);
      handles.cam.GevSCPD = delay;
      handles.cam.Timeout = 100;
      set(handles.ed_Exposition,'string',num2str(handles.cam.ExposureTime)); 
catch
      display('Problems with camera reinitialisation');
end
guidata(hObject, handles);



function edVolts_Callback(hObject, eventdata, handles)
% hObject    handle to edVolts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edVolts as text
%        str2double(get(hObject,'String')) returns contents of edVolts as a double
Vout = str2num(get(hObject,'string'));

RH = find_RH(Vout,handles.T(end,2),'210');
set(handles.edHumidity,'string',num2str(RH));

% --- Executes during object creation, after setting all properties.
function edVolts_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edVolts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edHumidity_Callback(hObject, eventdata, handles)
% hObject    handle to edHumidity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edHumidity as text
%        str2double(get(hObject,'String')) returns contents of edHumidity as a double


% --- Executes during object creation, after setting all properties.
function edHumidity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edHumidity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
