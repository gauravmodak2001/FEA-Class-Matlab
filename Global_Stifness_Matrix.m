function Global_elements_structure = Global_Stifness_Matrix(convert_to_index, elements_structure, data)
% PSEUDOCODE:
% 1. Extract connectivity matrix, total DOFs, and DOF per node from data
% 2. Initialize global stiffness matrix K and force vector F
% 3. For each element:
%    a. Get element stiffness matrix and surface force vector
%    b. Map element DOFs to global DOFs using connectivity matrix
%    c. Assemble element contributions into global K and F
% 4. Return global K and F in output structure

connectivityMatrix = data.Connectivity;  % Get element-node connectivity
nTotalDof = data.nTotalDOF;              % Total degrees of freedom in model
nDOFPNODE = data.nDOFPNode;              % DOFs per node (typically 2 for 2D)
[numElements, num_elemental_nodes] = size(connectivityMatrix);  % Get number of elements and nodes per element

K = zeros(nTotalDof, nTotalDof);  % Initialize global stiffness matrix: K_global
F_s = zeros(nTotalDof, 1);        % Initialize global force vector: F_global

% Initialize output structure
Global_elements_structure(1).K = [];
Global_elements_structure(1).F = [];

% Assembly loop - implements: K_global = Σ_e LᵉᵀK_e Lᵉ
for i = 1:numElements
    K_elements = elements_structure(i).Stiffnessmatrix;          % Get element stiffness matrix K_e
    F_Surface_Force_Elements = elements_structure(i).SurfaceForce;  % Get element force vector F_e
    
    % Loop through element nodes to populate global matrices
    for j = 1:num_elemental_nodes
        for k = 1:nDOFPNODE
            % Calculate row indices for element and global matrices
            elemental_row = convert_to_index(j, k, nDOFPNODE);                  % Local DOF index
            Local_row_Node = connectivityMatrix(i, j);                          % Global node number
            Local_Row = convert_to_index(Local_row_Node, k, nDOFPNODE);         % Global DOF index
            
            % Assemble force: F_global(I) += F_e(i)
            F_s(Local_Row) = F_s(Local_Row) + F_Surface_Force_Elements(elemental_row);
            
            % Loop to populate stiffness matrix
            for l = 1:num_elemental_nodes
                for m = 1:nDOFPNODE
                    % Calculate column indices for element and global matrices
                    elemental_col = convert_to_index(l, m, nDOFPNODE);           % Local DOF index
                    Local_Col_Node = connectivityMatrix(i, l);                   % Global node number
                    Local_Col = convert_to_index(Local_Col_Node, m, nDOFPNODE);  % Global DOF index
                    
                    % Assemble stiffness: K_global(I,J) += K_e(i,j)
                    K(Local_Row, Local_Col) = K(Local_Row, Local_Col) + K_elements(elemental_row, elemental_col);
                end
            end
        end
    end
end

% Store assembled matrices in output structure
Global_elements_structure.K = K;
Global_elements_structure.F = F_s;
end