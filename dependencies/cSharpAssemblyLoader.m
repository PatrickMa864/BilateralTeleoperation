classdef cSharpAssemblyLoader < handle
    %CSHARPASSEMBLYLOADER Gives current Matlab program acces to C# assemblies in a
    %folder
    
    properties
        dllFileInfo
    end
    
    methods
        
         function dllFileInfo = packageInfo(obj)
            % PACKAGEINFO Information about the C# packages used to
            % communicate with the robot
            %   A cell array, with each element corresponding to a package,
            %   each containing information about all the classes, methods
            %   and structures in that package
            dllFileInfo=obj.dllFileInfo;
         end
    end
    
    methods (Access = protected)
        
        function dllFileName = setupCSharpEnv(obj, cSharpDllFilesFolderPath, silent, utilPackagesList)
            % SETUPCSHARPENV Makes all the C# dlls known to Matlab and creates list of all classes/packages
            %
            % SETUPCSHARPENV(CSHARPDLLFILESFOLDERPATH, SILENT) will discover all C# dll files in the CSHARPDLLFILESFOLDERPATH, and list all dll files found if SILENT is true
            listDll = dir(cSharpDllFilesFolderPath);

            [m n] = size(listDll);

            obj.dllFileInfo = {};
            dllFileName = {};
            for i = 1:m
                fileName = listDll(i).name;

                indexes = strfind(fileName, '.dll');
                if ~isempty(indexes)
                    info = NET.addAssembly([cSharpDllFilesFolderPath, '\', fileName]);
                    obj.dllFileInfo{end+1} = info;
                    dllFileName{end+1} = fileName(1:indexes(1)-1);

                end
            end


            for i = 1:size(utilPackagesList,2)
                obj.dllFileInfo{end+1} = NET.addAssembly(utilPackagesList{i});
                dllFileName{end+1} = utilPackagesList{i};
            end

            if ~silent

                disp('C# assemblies are now known by Matlab');
                disp('Here is a list of the different C# packages discovered:');
                fprintf('\n');

                for i = 1:size(dllFileName,2)
                    disp(dllFileName{i});
                end
                fprintf('\n');
                
            end
            
        end
        
        function enumValue = getCSharpEnumValue(obj, enum, enumMemberString)
            allNames = System.Enum.GetNames(enum.GetType);
            allValues = System.Enum.GetValues(enum.GetType);
            for i=1:allNames.Length
                if strcmp(char(allNames(i)),enumMemberString)
                    break;
                end
            end
            enumValue = allValues(i);
        end
    end
end

