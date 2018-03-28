function varargout = OrtungV8txt(varargin)
% ORTUNGV8TXT MATLAB code for OrtungV8txt.fig
%      ORTUNGV8TXT, by itself, creates a new ORTUNGV8TXT or raises the existing
%      singleton*.
%
%      H = ORTUNGV8TXT returns the handle to a new ORTUNGV8TXT or the handle to
%      the existing singleton*.
%
%      ORTUNGV8TXT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ORTUNGV8TXT.M with the given input arguments.
%
%      ORTUNGV8TXT('Property','Value',...) creates a new ORTUNGV8TXT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before OrtungV8txt_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to OrtungV8txt_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help OrtungV8txt

% Last Modified by GUIDE v2.5 19-Jun-2017 12:26:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @OrtungV8txt_OpeningFcn, ...
                   'gui_OutputFcn',  @OrtungV8txt_OutputFcn, ...
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


% --- Executes just before OrtungV8txt is made visible.
function OrtungV8txt_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to OrtungV8txt (see VARARGIN)

% Choose default command line output for OrtungV8txt
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes OrtungV8txt wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1

% --- Executes during object creation, after setting all properties.
function FileView_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FileView (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Outputs from this function are returned to the command line.
function varargout = OrtungV8txt_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in LoadButton.
function LoadButton_Callback(hObject, eventdata, handles)
% hObject    handle to LoadButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%
[FileName,PathName,FilterIndex]=uigetfile({'*.txt','Text Files'},'File Selector');

FileView=FileName;

set(handles.FileView,'String',FileView);


       
% Lesen & Transformieren
% =================================================================
if (FileName ~= 0)

ld = msgbox('Lese Text Datei......','','Warn');

DATA = importTXTb(FileName);

delete(ld);

DataAmount=length(DATA.Quat_w);
set(handles.DataAmount,'String',DataAmount);

rotm = quat2dcm([DATA.Quat_w DATA.Quat_x DATA.Quat_y DATA.Quat_z]);

if (isempty(DATA.Altitude) == 0)

Alt = DATA.Altitude;

else
    
waitfor(errordlg('Keine Höhendaten vorhanden!','File Error'));
Alt=zeros(length(rotm),1);
 
end

%------------Einheitsvektor/Nulllage------------

pos=zeros(3,1); 


%-----------Vektor Plot (mit Waitbar)-------------------------------
%-----------Erstellen Waitbar Bedingungen---------------------------

wb = waitbar(0,'Berechnung läuft...','CreateCancelBtn','setappdata(gcbf,''cancel'',1)');
wbend = length(rotm)-1;
canceled = 0;

%-----------For-Schleife für Integration----------------------------
b=0;
Abrolldist=2400/8;

for i=1:length(rotm)-1

    if getappdata(wb,'cancel')
        canceled = 1;
        break;
    end
     
     a = rotm(:,:,i)*[DATA.HallSensorData(i)*Abrolldist/1000;0;0];
     b = b+(DATA.HallSensorData(i)*Abrolldist);
     
     pos(:,i+1)= pos(:,i)+a(:,1);
     
     waitbar(i/wbend);


end

%----------Darstellung über Plot-----------------------------------

axes(handles.axes1);
plot3(pos(1,:),pos(2,:), Alt(:));
% zlim([-5 5]);
rotate3d on;
box on;
grid on;
xlabel('X-Achse in Metern');
ylabel('Y-Achse in Metern');
zlabel('Z-Achse in Metern');
delete(wb);

%----------Distanzanzeige-----------------------------------------

c = b/1000;
cn = num2str(c);
Distance=cn;
set(handles.Distance,'String',Distance);


%----------Hinweisboxen--------------------------------------------

if (canceled == 1)
   h = msgbox('Vorgang vorzeitig abgebrochen!', '','error');
else
   h = msgbox('Vorgang abgeschlossen!','','Help');
end

end

%----------KML Write-----------------------------------------------


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[FileName,PathName,FilterIndex]=uigetfile({'*.txt','Text Files'},'File Selector');

if (FileName ~= 0)

ld = msgbox('Lese Text Datei......','','Warn');

DATA = importTXTb(FileName);

delete(ld);

if (DATA.GooglemapsLocation_lat ~= 0)

kmlname = 'GPStoKML.kml';

kmlwrite(kmlname,DATA.GooglemapsLocation_lat,DATA.GooglemapsLocation_long);

h = msgbox('KML Daten erstellt!','','Help');

else
    
    waitfor(errordlg('Keine GPS Daten vorhanden!','File Error'));
end
end
    
% --- Executes on button press in Exitbutton.
function Exitbutton_Callback(hObject, ~, handles)
% hObject    handle to Exitbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

choice = questdlg('Wirklich beenden?','Programm beenden','Ja','Nein','Abbruch','Abbruch');

switch choice
    case 'Ja'
      close all
    otherwise        
end
