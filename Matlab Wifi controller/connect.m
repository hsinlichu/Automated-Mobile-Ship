function varargout = connect(varargin)
% CONNECT MATLAB code for connect.fig
%      CONNECT, by itself, creates a new CONNECT or raises the existing
%      singleton*.
%
%      H = CONNECT returns the handle to a new CONNECT or the handle to
%      the existing singleton*.
%
%      CONNECT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CONNECT.M with the given connect arguments.
%
%      CONNECT('Property','Value',...) creates a new CONNECT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before connect_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to connect_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help connect

% Last Modified by GUIDE v2.5 27-May-2018 22:41:41

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @connect_OpeningFcn, ...
    'gui_OutputFcn',  @connect_OutputFcn, ...
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


% --- Executes just before connect is made visible.
function connect_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to connect (see VARARGIN)

% Choose default command line output for connect
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes connect wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = connect_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function set_ip_Callback(hObject, eventdata, handles)
% hObject    handle to set_ip (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of set_ip as text
%        str2double(get(hObject,'String')) returns contents of set_ip as a double


% --- Executes during object creation, after setting all properties.
function set_ip_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_ip (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function set_port_Callback(hObject, eventdata, handles)
% hObject    handle to set_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of set_port as text
%        str2double(get(hObject,'String')) returns contents of set_port as a double


% --- Executes during object creation, after setting all properties.
function set_port_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in submit.
function submit_Callback(hObject, eventdata, handles)
% hObject    handle to submit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ip = get(handles.set_ip,'String');
port = str2num(get(handles.set_port,'String'));
tmp = sprintf('%s : %d',ip,port);
tmp2 = sprintf('Connecting to %s . . . ',tmp);
f = waitbar(.33,tmp2,'Name','Connection');
try
    t = tcpip(ip,port);
    fopen(t);
catch
    ErrorMessage=lasterr;
    close(f);
    errordlg(ErrorMessage,'Error');
    return;
end
waitbar(1,f);
close(f);
%disp(ip);
%disp(port);
%close(connect);
CarController('Tag','figure1',tmp,t);


% --- Executes on key press with focus on figure1 or any of its controls.
function figure1_WindowKeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
key = eventdata.Key;
if(strcmp(key,'return'))
    submit_Callback(hObject, eventdata, handles);
end
