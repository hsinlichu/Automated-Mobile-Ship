function varargout = CarController(varargin)
% CARCONTROLLER MATLAB code for CarController.fig
%      CARCONTROLLER, by itself, creates a new CARCONTROLLER or raises the existing
%      singleton*.
%
%      H = CARCONTROLLER returns the handle to a new CARCONTROLLER or the handle to
%      the existing singleton*.
%
%      CARCONTROLLER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CARCONTROLLER.M with the given input arguments.
%
%      CARCONTROLLER('Property','Value',...) creates a new CARCONTROLLER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before CarController_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to CarController_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help CarController

% Last Modified by GUIDE v2.5 13-Jun-2018 20:30:22

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @CarController_OpeningFcn, ...
    'gui_OutputFcn',  @CarController_OutputFcn, ...
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


% --- Executes just before CarController is made visible.
function CarController_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to CarController (see VARARGIN)

% Choose default command line output for CarController
handles.output = hObject;
disp('OpeningFcn');

set(handles.text9,'String',varargin{3});
handles.t = varargin{4};
guidata(hObject, handles);
handles.last = 'P';
guidata(hObject, handles);
set(handles.text11,'String',handles.t.status);
disp(handles.t.status);

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes CarController wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = CarController_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Forward.
function Forward_Callback(hObject, eventdata, handles)
% hObject    handle to Forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.Status, 'String', 'Forward');
tmp = 'f';
fprintf(handles.t,"%c",tmp);
flushinput(handles.t);



% --- Executes on button press in Backward.
function Backward_Callback(hObject, eventdata, handles)
% hObject    handle to Backward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.Status, 'String', 'Backward');
tmp = 'b';
fprintf(handles.t,"%c",tmp);
flushinput(handles.t);

% --- Executes on button press in Stop.
function Stop_Callback(hObject, eventdata, handles)
% hObject    handle to Stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.Status, 'String', 'Stop');
tmp = 'x';
fprintf(handles.t,"%c",tmp);
flushinput(handles.t);

% --- Executes on button press in Right.
function Right_Callback(hObject, eventdata, handles)
% hObject    handle to Right (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.Status, 'String', 'Right');
tmp = 'r';
fprintf(handles.t,"%c",tmp);
flushinput(handles.t);

% --- Executes on button press in Left.
function Left_Callback(hObject, eventdata, handles)
% hObject    handle to Left (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.Status, 'String', 'Left');
tmp = 'l';
fprintf(handles.t,"%c",tmp);
flushinput(handles.t);

% --- Executes on key press with focus on figure1 or any of its controls.
function figure1_WindowKeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

if(strcmp(eventdata.Key,'uparrow'))
    tmp = get(handles.slider1,'Value');
    if(tmp < 5)
        set(handles.slider1, 'Value', tmp + 1);                  %set slider position to rounded off value
        guidata(hObject, handles);
        slider1_Callback(hObject, eventdata, handles)
    end
    
elseif(strcmp(eventdata.Key,'downarrow'))
    tmp = get(handles.slider1,'Value');
    if(tmp > 0)
        set(handles.slider1, 'Value', tmp - 1);                  %set slider position to rounded off value
        guidata(hObject, handles);
        slider1_Callback(hObject, eventdata, handles)
    end
elseif(eventdata.Key ~= handles.last)
    disp("press:" + eventdata.Key);
    switch eventdata.Key
        case 'w'
            Forward_Callback(hObject, eventdata, handles);
            handles.front = 1;
        case 'a'
            Left_Callback(hObject, eventdata, handles);
        case 's'
            Backward_Callback(hObject, eventdata, handles);
        case 'd'
            Right_Callback(hObject, eventdata, handles);
        case 'x'
            Stop_Callback(hObject, eventdata, handles);
    end
    handles.last = eventdata.Key;
    guidata(hObject, handles);
end
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
disp('CreateFcn');


% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp('DeleteFcn');
echotcpip('off');
fclose(handles.t);


% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(get(hObject,'Value'))
    tmp = 'm';
else
    tmp = 'c';
end
fprintf(handles.t,"%c",tmp);
flushinput(handles.t);

% Hint: get(hObject,'Value') returns toggle state of radiobutton2

% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
newval = handles.slider1.Value;                         %get value from the slider
newval = round(newval);                         %round off this value
set(handles.slider1, 'Value', newval);                  %set slider position to rounded off value
handles.speed = get(handles.slider1,'Value');
disp(['Slider moved to ' num2str(newval)]);     %display the value pointed by slider
guidata(hObject, handles);
fprintf(handles.t,"%d",newval);
flushinput(handles.t);



% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonUpFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on key release with focus on figure1 or any of its controls.
function figure1_WindowKeyReleaseFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was released, in lower case
%	Character: character interpretation of the key(s) that was released
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) released
% handles    structure with handles and user data (see GUIDATA)
if(get(handles.radiobutton2,'Value') == 0)
    disp("release:" + eventdata.Key);
    switch eventdata.Key
        case 'w'
            Stop_Callback(hObject, eventdata, handles);
            handles.front = 0;
        case 'a'
            if(handles.front)
                Forward_Callback(hObject, eventdata, handles);
            else
                Stop_Callback(hObject, eventdata, handles);
            end
        case 's'
            Stop_Callback(hObject, eventdata, handles);
        case 'd'
            if(handles.front)
                Forward_Callback(hObject, eventdata, handles);
            else
                Stop_Callback(hObject, eventdata, handles);
            end
        case 'x'
            Stop_Callback(hObject, eventdata, handles);
    end
    handles.last = 'P';
    guidata(hObject, handles);
end
