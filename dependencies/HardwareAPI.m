classdef HardwareAPI < handle & cSharpAssemblyLoader
    % HardwareAPI Wrapper around the Haply C# hardware API
    
    properties (Constant)
        dof = 3; % Number of degrees of freedom of the Inverse3
    end
    
    properties (Access = private)
        port
        connection
        cnvrtToMtlbArrayByDefault
    end
    
    properties (SetAccess = immutable)
        deviceId
        deviceModelNumber
        deviceHardwareVersion
        deviceFirmwareVersion
    end
    
    methods
        function obj = HardwareAPI(comPort, silent, cnvrtToMtlbArrayByDefault)
            % CONSTRUCTOR Sets up robot to be ready for use 
            %   Makes the different C# assemblies needed for comunication
            %   with the robot visible to MATLAB
            %   Connects to the robot
            
            if nargin < 3
                cnvrtToMtlbArrayByDefault = false;
            end
            if nargin < 2
                silent = false;
            end
            obj.cnvrtToMtlbArrayByDefault = cnvrtToMtlbArrayByDefault;
            
            currentFolder = pwd;
            cSharpDllFilesFolderPath = '\HardwareAPI\net4.8\Newtonsoft.Json.13.0.0.0\Release';
            cSharpDllFilesFolderPath = join([currentFolder, cSharpDllFilesFolderPath]);
            obj.setupCSharpEnv(cSharpDllFilesFolderPath, silent);

            import System.IO.Ports.*
            import Haply.HardwareAPI.Devices.*

            obj.port = SerialPort(comPort);
            obj.port.Open();
            obj.port.DiscardInBuffer();
            obj.port.DiscardOutBuffer();
            stream = obj.port.BaseStream;

            obj.connection = Inverse3Connection(stream);
            
            deviceInfoResponse = obj.DeviceWakeup();
            obj.deviceId = deviceInfoResponse.deviceId;
            obj.deviceModelNumber = deviceInfoResponse.deviceModelNumber;
            obj.deviceHardwareVersion = deviceInfoResponse.deviceHardwareVersion;
            obj.deviceFirmwareVersion = deviceInfoResponse.deviceFirmwareVersion;
            
        end
        
        function delete(obj)
            % DELETE Stops comunication with the Inverse3 robot
            obj.port.Close();
        end
        
        function deviceInfoResponse = DeviceWakeup(obj)
           deviceInfoResponse = obj.connection.DeviceWakeup(); 
        end
        
        % NOTE: To use in a new method the same structure/syntax found in  
        % the functions up to the NOTE_END comment, make sure the method
        % name has the same exact name as the C# request method in the C#
        % Inverse3Connection class. For example, JointTorques,
        % EndEffectorPosition, DevicePower MotorCurrents, etc., are all
        % found in the Inverse3Connection.cs file and the equivalent 
        % methods here will return the same structs if 
        % cnvrtRspnseToMtlbArray is false.
        
        function varargout = JointTorques(obj, request, cnvrtRspnseToMtlbArray)
            % JOINTTORQUES Request the Inverse3 apply a certain torque on each joint
            %
            % JOINTTORQUES(REQUEST, CNVRTRSPNSETOMATLABARRAY) applies a certain torque on each joint and returns either two arrays or a single object containing information about joint angles and angular velocities, depending on either if CNVRTRSPNSETOMATLABARRAY is true or CNVRTRSPNSETOMATLABARRAY is false. We have thus either [angles, angularVelocities] = JOINTTORQUES(REQUEST, true) or response = JOINTTORQUES(REQUEST, false)
            %
            % JOINTTORQUES(REQUEST) applies a certain torque on each joint and returns either two arrays or a single object containing information about joint angles and and angular velocities, depending on either if cnvrtToMtlbArrayByDefault is set as true in the constructor or cnvrtToMtlbArrayByDefault is set as false
            
            if nargin < 3
                cnvrtRspnseToMtlbArray = obj.cnvrtToMtlbArrayByDefault;
            end
            % Next two lines find the current method name
            cllstck = dbstack;
            type = extractAfter(cllstck(1).name, '.');
            varargout = obj.requestToResponseBaseline(request, type, cnvrtRspnseToMtlbArray, nargout);
        end
        
        function varargout = JointAngles(obj, request, cnvrtRspnseToMtlbArray)
            % JOINTANGLES Request the Inverse3 move to a certain angle on each joint
            %
            % JOINTANGLES(REQUEST, CNVRTRSPNSETOMATLABARRAY) move to a certain angle on each joint and returns either two arrays or a single object containing information about joint angles and angular velocities, depending on either if CNVRTRSPNSETOMATLABARRAY is true or CNVRTRSPNSETOMATLABARRAY is false. We have thus either [angles, angularVelocities] = JOINTANGLES(REQUEST, true) or response = JOINTANGLES(REQUEST, false)
            %
            % JOINTANGLES(REQUEST) move to a certain angle on each joint and returns either two arrays or a single object containing information about joint angles and and angular velocities, depending on either if cnvrtToMtlbArrayByDefault is set as true in the constructor or cnvrtToMtlbArrayByDefault is set as false
            
            if nargin < 3
                cnvrtRspnseToMtlbArray = obj.cnvrtToMtlbArrayByDefault;
            end
            % Next two lines find the current method name
            cllstck = dbstack;
            type = extractAfter(cllstck(1).name, '.');
            varargout = obj.requestToResponseBaseline(request, type, cnvrtRspnseToMtlbArray, nargout);
        end
        
        function varargout = EndEffectorForce(obj, request, cnvrtRspnseToMtlbArray)
            % ENDEFFECTORFORCE Request the Inverse3 to apply a certain force on the end effector
            %
            % ENDEFFECTORFORCE(REQUEST, CNVRTRSPNSETOMATLABARRAY) to apply a certain force on the end effector and returns either two arrays or a single object containing information about end effector position and velocity, depending on either if CNVRTRSPNSETOMATLABARRAY is true or CNVRTRSPNSETOMATLABARRAY is false. We have thus either [angles, angularVelocities] = ENDEFFECTORFORCE(REQUEST, true) or response = ENDEFFECTORFORCE(REQUEST, false)
            %
            % ENDEFFECTORFORCE(REQUEST) to apply a certain force on the end effector and returns either two arrays or a single object containing information about end effector position and velocity, depending on either if cnvrtToMtlbArrayByDefault is set as true in the constructor or cnvrtToMtlbArrayByDefault is set as false
            
            if nargin < 3
                cnvrtRspnseToMtlbArray = obj.cnvrtToMtlbArrayByDefault;
            end
            % Next two lines find the current method name
            cllstck = dbstack;
            type = extractAfter(cllstck(1).name, '.');
            varargout = obj.requestToResponseBaseline(request, type, cnvrtRspnseToMtlbArray, nargout);
        end
        
        function varargout = EndEffectorPosition(obj, request, cnvrtRspnseToMtlbArray)
            % ENDEFFECTORPOSITION Request the Inverse3 to move the end effector to a certain position
            %
            % ENDEFFECTORPOSITION(REQUEST, CNVRTRSPNSETOMATLABARRAY) to move the end effector to a certain position and returns either two arrays or a single object containing information about end effector position and velocity, depending on either if CNVRTRSPNSETOMATLABARRAY is true or CNVRTRSPNSETOMATLABARRAY is false. We have thus either [angles, angularVelocities] = ENDEFFECTORPOSITION(REQUEST, true) or response = ENDEFFECTORPOSITION(REQUEST, false)
            %
            % ENDEFFECTORPOSITION(REQUEST) to move the end effector to a certain position and returns either two arrays or a single object containing information about end effector position and velocity, depending on either if cnvrtToMtlbArrayByDefault is set as true in the constructor or cnvrtToMtlbArrayByDefault is set as false
            
            if nargin < 3
                cnvrtRspnseToMtlbArray = obj.cnvrtToMtlbArrayByDefault;
            end
            % Next two lines find the current method name
            cllstck = dbstack;
            type = extractAfter(cllstck(1).name, '.');
            varargout = obj.requestToResponseBaseline(request, type, cnvrtRspnseToMtlbArray, nargout);
        end
        
        function response = DeviceOrientation(obj, cnvrtRspnseToMtlbArray)
            % DEVICEORIENTATION Request information about the the device's orientation
            %
            % DEVICEORIENTATION(CNVRTRSPNSETOMATLABARRAY) Request information about the the device's orientation and returns either an arrays or a single object containing information about the device's orientation, depending on either if CNVRTRSPNSETOMATLABARRAY is true or CNVRTRSPNSETOMATLABARRAY is false
            %
            % DEVICEORIENTATION() Request information about the the device's orientation and returns either an array or a single object containing information about the device's orientation, depending on either if cnvrtToMtlbArrayByDefault is set as true in the constructor or cnvrtToMtlbArrayByDefault is set as false
            
            if nargin < 2
                cnvrtRspnseToMtlbArray = obj.cnvrtToMtlbArrayByDefault;
            end
            % Next two lines find the current method name
            cllstck = dbstack;
            type = extractAfter(cllstck(1).name, '.');
            response = obj.simpleRequestToResponseBaseline(type, cnvrtRspnseToMtlbArray);
        end
        
        function response = DevicePower(obj, cnvrtRspnseToMtlbArray)
            % DEVICEPOWER Request information about the the device's power
            %
            % DEVICEPOWER(CNVRTRSPNSETOMATLABARRAY) Request information about the the device's power and returns either an arrays or a single object containing information about the device's power, depending on either if CNVRTRSPNSETOMATLABARRAY is true or CNVRTRSPNSETOMATLABARRAY is false
            %
            % DEVICEPOWER() Request information about the the device's power and returns either an array or a single object containing information about the device's power, depending on either if cnvrtToMtlbArrayByDefault is set as true in the constructor or cnvrtToMtlbArrayByDefault is set as false
            
            if nargin < 2
                cnvrtRspnseToMtlbArray = obj.cnvrtToMtlbArrayByDefault;
            end
            % Next two lines find the current method name
            cllstck = dbstack;
            type = extractAfter(cllstck(1).name, '.');
            response = obj.simpleRequestToResponseBaseline(type, cnvrtRspnseToMtlbArray);
        end
        
        function response = DeviceTemperature(obj, cnvrtRspnseToMtlbArray)
            % DEVICETEMPERATURE Request information about the the device's temperature
            %
            % DEVICETEMPERATURE(CNVRTRSPNSETOMATLABARRAY) Request information about the the device's temperature and returns either an arrays or a single object containing information about the device's temperature, depending on either if CNVRTRSPNSETOMATLABARRAY is true or CNVRTRSPNSETOMATLABARRAY is false
            %
            % DEVICETEMPERATURE() Request information about the the device's temperature and returns either an array or a single object containing information about the device's temperature, depending on either if cnvrtToMtlbArrayByDefault is set as true in the constructor or cnvrtToMtlbArrayByDefault is set as false
            
            if nargin < 2
                cnvrtRspnseToMtlbArray = obj.cnvrtToMtlbArrayByDefault;
            end
            % Next two lines find the current method name
            cllstck = dbstack;
            type = extractAfter(cllstck(1).name, '.');
            response = obj.simpleRequestToResponseBaseline(type, cnvrtRspnseToMtlbArray);
        end
        
        function response = MotorCurrents(obj, cnvrtRspnseToMtlbArray)
            % MOTORCURRENTS Request information about the the device's motors' currents.
            %
            % MOTORCURRENTS(CNVRTRSPNSETOMATLABARRAY) Request information about the the device's motors' currents and returns either an arrays or a single object containing information about the device's motors' currents, depending on either if CNVRTRSPNSETOMATLABARRAY is true or CNVRTRSPNSETOMATLABARRAY is false
            %
            % MOTORCURRENTS() Request information about the the device's motors' currents and returns either an array or a single object containing information about the device's motors' currents, depending on either if cnvrtToMtlbArrayByDefault is set as true in the constructor or cnvrtToMtlbArrayByDefault is set as false
            
            if nargin < 2
                cnvrtRspnseToMtlbArray = obj.cnvrtToMtlbArrayByDefault;
            end
            % Next two lines find the current method name
            cllstck = dbstack;
            type = extractAfter(cllstck(1).name, '.');
            response = obj.simpleRequestToResponseBaseline(type, cnvrtRspnseToMtlbArray);
        end
        
        % NOTE_END
    end
    
    methods (Static)
        function request = generateBlankJointTorquesRequest()
            % GENERATEBLANKJOINTTORQUESREQUEST Returns a blank, newly constructed, C# request for specific joint torques
            import Haply.HardwareAPI.Runtime.Generated.*
            request = JointTorquesRequest();
        end
        
        function request = generateBlankJointAnglesRequest()
            % GENERATEBLANKJOINTANGLESREQUEST Returns a blank, newly constructed, C# request for specific joint angles
            import Haply.HardwareAPI.Runtime.Generated.*
            request = JointAnglesRequest();
        end
        
        function request = generateBlankEndEffectorForceRequest()
            % GENERATEBLANKENDEFFECTORFORCEREQUEST Returns a blank, newly constructed, C# request for specific end effector force
            import Haply.HardwareAPI.Runtime.Generated.*
            request = EndEffectorForceRequest();
        end
        
        function request = generateBlankEndEffectorPositionRequest()
            % GENERATEBLANKENDEFFECTORPOSITIONREQUEST Returns a blank, newly constructed, C# request for specific end effector position
            import Haply.HardwareAPI.Runtime.Generated.*
            request = EndEffectorForceRequest();
        end
    end
    
    methods (Access = protected)
        
        function setupCSharpEnv(obj, cSharpDllFilesFolderPath, silent)
            % SETUPCSHARPENV Makes all the C# dlls known to Matlab and creates list of all classes/packages
            %
            % SETUPCSHARPENV(CSHARPDLLFILESFOLDERPATH, SILENT) will discover all C# dll files in the CSHARPDLLFILESFOLDERPATH, and list all dll files found if SILENT is true
            

            utilPackagesList = {'System'};
            
            setupCSharpEnv@cSharpAssemblyLoader(obj, cSharpDllFilesFolderPath, silent, utilPackagesList);

            if ~silent
                disp('To facilitate use of any of these packages, follow the next example: ');
                fprintf('\n');
                disp('To alleviate the need to write')
                fprintf('\n');
                disp(' > request =  Haply.HardwareAPI.Runtime.Generated.EndEffectorForceRequest();');
                fprintf('\n');
                disp('everytime a force request struct needs to be created, ');
                disp('call in scope of your code (see import doc if need be), the following: ');
                fprintf('\n');
                disp(' > import Haply.HardwareAPI.Runtime.Generated.*  ');
                fprintf('\n');
                disp('This will considerably reduce the code to generate a force request struct: ');
                fprintf('\n');
                disp(' > request = EndEffectorForceRequest(); ');
                fprintf('\n');
                disp('To see what functions and classes are available in each package, please consult');
                disp('the contents of the dllFileInfo cell array returned by the packageInfo method.');
                disp('of this class. Each element of the cell array corresponds to one package.');
%                 fprintf('\n');
%                 disp('NOTE: some attributes or method arguments in the C# classes are single precision');
%                 disp('floating point numbers. Make sure to convert any Matlab floating point');
%                 disp('number to single precision when this is the case, as Matlab floating point '); 
%                 disp('numbers are double precision by default.');
            end

        end
    end
    
    methods (Access = private)
        function correctRequest = correctRequestType(obj, request, typeName, vectorSize)
            % CORRECTREQUESTTYPE Makes sure the request that will be passed
            % to a C# request caller is a C# object
            %
            % correctRequest = CORRECTREQUESTTYPE(REQUEST, TYPENAME,
            % VECTORSIZE) makes sure that the REQUEST object is converted,
            % if necessary, to the
            % Haply.HardwareAPI.Runtime.Generated.TYPENAME.
            % When converting, a Matlab array being converted to a C#
            % object must be the correct size specified by VECTORSIZE
            completeTypeName = ['Haply.HardwareAPI.Runtime.Generated.', typeName];
            if ~isequal(class(request), completeTypeName)
                if ~((size(request, 1) == 1 && size(request, 2) == vectorSize) || (size(request, 1) == vectorSize && size(request, 2) == 1))
                    error([typeName, ' must always be a vector containing ', num2str(vectorSize),' elements']);
                end
                if isequal(class(request), 'double')
                    correctRequest = obj.convertMatlabArrayToCSharpRequest(single(request), completeTypeName);
                elseif isequal(class(request), 'single')
                    correctRequest = obj.convertMatlabArrayToCSharpRequest(request, completeTypeName);
                else
                    error(['A ', typeName, ' must always contain floating point numbers.']);
                end
            else
                correctRequest = request;
            end
        end
        
        function cSharpRequest = convertMatlabArrayToCSharpRequest(obj, matlabArray, completeTypeName)
            % Takes a matlab array, and assigns the elements of that array,
            % in order, to the attributes of an object the type of which is
            % specified by the completeTypeName argument
            props = properties(completeTypeName);
            cSharpRequest = feval(completeTypeName);
            for iprop = 1:length(props)
              thisprop = props{iprop};
              cSharpRequest.(thisprop) = matlabArray(iprop);
            end
        end
        
        function matlabArray = convertCSharpResponseToMatlabArray(obj, cSharpArrayLike)
           % Takes a C# response object and creates an array filled with
           % the late's object attributes, in order
            props = properties(cSharpArrayLike);
            matlabArray = zeros(size(props,2),1);
            for iprop = 1:length(props)
              thisprop = props{iprop};
              thisprop_value = cSharpArrayLike.(thisprop);
              matlabArray(iprop,1) = thisprop_value;
            end
        end
        
        function response = requestToResponseBaseline(obj, request, type, cnvrtRspnseToMtlbArray, nOutputs)
            % Calls a request method of the connection (Inverse3Connection) C#
            % object of this class, using the given request object to
            % specify what that method should do exactly. The method called
            % is specified using the type argument.
            % The cnvrtRspnseToMtlbArray specifies if the response returned
            % by that C# request method should stay in it's original C#
            % object format or be converted to a matlab array, for ease of
            % use in Matlab
            request = obj.correctRequestType(request, [type, 'Request'], obj.dof);
            response = feval(type, obj.connection, request);
            if cnvrtRspnseToMtlbArray
                if nOutputs ~= 2
                    error(['A ', type, ' response is always split in two equal length arrays when working in Matlab array format']);
                end
                response = obj.splitMatlabArrayResponse(obj.convertCSharpResponseToMatlabArray(response));
            else
                if nOutputs ~= 1
                    error(['A ', type, ' response is always is always a single struct when working with the default CSharp dafault types']);
                end
                response = {response};
            end
        end
        
        function response = simpleRequestToResponseBaseline(obj, type, cnvrtRspnseToMtlbArray)
            % Calls a request method of the connection (Inverse3Connection) C#
            % object of this class. The method called
            % is specified using the type argument.
            % The cnvrtRspnseToMtlbArray specifies if the response returned
            % by that C# request method should stay in it's original C#
            % object format or be converted to a matlab array, for ease of
            % use in Matlab
            response = feval(type, obj.connection);
            if cnvrtRspnseToMtlbArray
                response = obj.convertCSharpResponseToMatlabArray(response);
            end
        end
        
        function responseCellArray = splitMatlabArrayResponse(obj, matlabArrayResponse)
            % Separates the matlab array response in two, equal lenght arrays 
            responseCellArray = cell(1,2);
            length = size(matlabArrayResponse,1);
            if mod(length,2) ~= 0
                error('Can only split odd length vectors in two equal parts');
            end
            responseCellArray{1} = matlabArrayResponse(1:length/2,1);
            responseCellArray{2} = matlabArrayResponse(length/2+1:end,1);
        end
        
    end
end

