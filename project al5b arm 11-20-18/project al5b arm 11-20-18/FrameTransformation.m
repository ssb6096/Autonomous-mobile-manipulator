classdef FrameTransformation
    %FrameTransformation Provides static methods to create rotational and
    %translational frame transformation matrices.
    %    
    %Authors:  Vu Nguyen        (vkn3505@rit.edu)
    %          Ryan M. Bowen    (rmb3518@rit.edu)
    %
    %Modified: 10/5/2011 (rmb3518)

    methods(Static)
        
        function frameReturn = trans(xyz, values)
        % Create a translation frame transformation matrix
        % 
        % @params   xyz     -   string list of translations i.e. 'y' or 'xy' or 'xyz'
        %           values  -   array of values for translation, must be the
        %                       same size in length as xyz string input.
        %
        % @returns  frameReturn - 4x4 frame transformation matrix.
            x = 0;
            y = 0;
            z = 0;
            for i = 1 : length(xyz)
                switch xyz(i)
                    case {'x'}
                        x = values(i);
                    case {'y'}
                        y = values(i);
                    case {'z'}
                        z = values(i);
                end
            end

            frameReturn =   [ 1 0 0 x;...
                              0 1 0 y;...
                              0 0 1 z;...
                              0 0 0 1];        
        end
        
        function frameReturn = rot(xyz,degree)
        % Create a rotational frame transformation matrix
        % 
        % @params   xyz     -   Axis of rotation i.e. 'x' or 'y' or 'z'
        %           values  -   value in degrees of the rotation
        %
        % @returns  frameReturn - 4x4 frame transformation matrix.
            
            C = cosd(degree);
            S = sind(degree);
            switch xyz
                case {'x'}
                    frameReturn = [ 1  0  0  0;...
                                    0  C -S  0;...
                                    0  S  C  0;...
                                    0  0  0  1];
                case {'y'}
                    frameReturn = [ C  0  S  0;...
                                    0  1  0  0;...
                                    -S 0  C  0;...
                                    0  0  0  1];
                case {'z'}
                    frameReturn = [ C  -S 0  0;...
                                    S  C  0  0;...
                                    0  0  1  0;...
                                    0  0  0  1];
            end
            
        end
    end
    
end

