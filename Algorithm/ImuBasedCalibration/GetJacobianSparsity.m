function JSparsity = GetJacobianSparsity(tRobot, tImu, y, d, C)
numMeasQ = length(tRobot);
numMeasZ = length(tImu);

dxl = [ones(3,1); zeros(3,1)];
dxr = ones(6,1);

delEzDelX = [dxl, dxr, repmat([dxl, dxl, dxr, dxr], 1, 4), repmat(dxl, 1, 3)];
delEzDelG = [ones(3,2); zeros(3,2)];
delEzDelTau = ones(6,1);

da = [0, 0, 0; 1, 0, 0; 0, 1, 1];
dr = [1, 1, 0; 1, 0, 1; 0, 1, 1];

delEzDelAlphA = [da; zeros(3)];
delEzDelRA = [dr; zeros(3)];
delEzDelKA = [eye(3); zeros(3)];
delEzDelBA = [eye(3); zeros(3)];

delEzDelAlphW = [zeros(3); da];
delEzDelRW = [zeros(3); dr];
delEzDelKW = [zeros(3); eye(3)];
delEzDelBW = [zeros(3); eye(3)];

delEzDelPi = [delEzDelX, delEzDelG, delEzDelTau, ...
              delEzDelAlphA, delEzDelRA, delEzDelKA, delEzDelBA, ...
              delEzDelAlphW, delEzDelRW, delEzDelKW, delEzDelBW];
          
delEzDelPi = repmat(sparse(delEzDelPi), numMeasZ, 1);
delEqDelPi = sparse(6*numMeasQ, 48);
delEpiDelPi = sparse(eye(48));

delEDelPi = [delEqDelPi; delEzDelPi; delEpiDelPi];

% Now deal with the bandedness caused by the local support of splines

% For joint limits, consider the full interval that newCols can change.
[numRows, numCols] = size(C);
c = C(:);

yIndC = reshape(repmat(1:numCols, numRows, 1), 1, []);
rowIndC = reshape(repmat(1:numRows, 1, numCols), 1, []);

ijQ = zeros(6*numMeasQ*length(c), 2);
ijZ = zeros(6*numMeasZ*length(c), 2);

numElemQ = 0;
numElemZ = 0;

% Build column by column
for jjj = 1:length(c)
    tSpanEffectQ = [y(yIndC(jjj)), y(yIndC(jjj) + d + 1)];
    
    indEffectT = find(and(tRobot >= tSpanEffectQ(1), tRobot <= tSpanEffectQ(2)));
    firstIndEffect = 6*(indEffectT - 1) + 1;
    indEffect = firstIndEffect + rowIndC(jjj) - 1;
    
    newRows = (numElemQ + 1):(numElemQ + length(indEffect));
    
    ijQ(newRows,:) = [indEffect', jjj*ones(size(indEffect'))];
    
    numElemQ = numElemQ + length(indEffect);
    
    
    % Now, we need a tolerance on the imu timing to account for the time
    % offset that we're calibrating.
    tImuTol = 0.5;
    tSpanEffectZ = tSpanEffectQ + [-tImuTol, tImuTol];
    
    indEffectT = find(and(tImu >= tSpanEffectZ(1), tImu <= tSpanEffectZ(2)));
    firstIndEffect = 6*(indEffectT - 1) + 1;
    indEffect = reshape([firstIndEffect + 0; 
                          firstIndEffect + 1; 
                          firstIndEffect + 2;
                          firstIndEffect + 3;
                          firstIndEffect + 4;
                          firstIndEffect + 5], 1, []);
    
    newRows = (numElemZ + 1):(numElemZ + length(indEffect));
    
    ijZ(newRows,:) = [indEffect', jjj*ones(size(indEffect'))];
    
    numElemZ = numElemZ + length(indEffect);
end

ijQ = ijQ(1:numElemQ,:);
ijZ = ijZ(1:numElemZ,:);

delEqDelC = sparse(ijQ(:,1), ijQ(:,2), ones(1, size(ijQ,1)), 6*numMeasQ, length(c));
delEzDelC = sparse(ijZ(:,1), ijZ(:,2), ones(1, size(ijZ,1)), 6*numMeasZ, length(c));

delEpiDelC = sparse(48, length(c));

delEDelC = [delEqDelC; delEzDelC; delEpiDelC];

JSparsity = [delEDelPi, delEDelC];
end