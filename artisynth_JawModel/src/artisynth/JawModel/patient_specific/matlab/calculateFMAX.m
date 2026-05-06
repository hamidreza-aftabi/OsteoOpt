%% compute_pcsa_fmax.m
% ------------------------------------------------------------
% INPUT FILES
%   SCSA.txt       -> whole-muscle SCS for left/right
%
% OUTPUT FILES
%   WPCSA.txt
%   BPCSA.txt
%   Final_PCSA.txt
%   FMAX.txt
%   Final_PCSA_average_log.txt
%
% ASSUMPTIONS
%   1) SCSA values are already in cm^2
%   2) Final_PCSA is the average of left/right WPCSA and BPCSA branch values
%   3) FMAX = Final_PCSA * 40
%   4) muscle branch proportions:
%        Masseter:           sm=0.70, dm=0.30
%        Medial pterygoid:   mp=1.00
%        Temporalis:         at=0.48, mt=0.29, pt=0.23
%        Lateral pterygoid:  ip=0.70, sp=0.30
% ------------------------------------------------------------

%% ===== FILE NAMES =====
scsFile = 'SCSA.txt';

%% ===== READ INPUT FILES =====
scsMap = readSCSAFile(scsFile);

%% ===== REGRESSION EQUATIONS =====
% Whole-muscle PCSA from whole-muscle SCS
% WPCSA = aW*SCS + bW
% BPCSA = aB*SCS + bB

reg = struct();
reg.M  = struct('aW',1.52,'bW', 1.04,'aB',1.11,'bB', 0.85);   % Masseter
reg.T  = struct('aW',2.45,'bW',-1.85,'aB',1.87,'bB',-1.51);   % Temporalis
reg.MP = struct('aW',2.34,'bW',-2.04,'aB',1.56,'bB',-1.40);   % Medial pterygoid
reg.LP = struct('aW',1.55,'bW',-3.42,'aB',0.93,'bB',-1.51);   % Lateral pterygoid

%% ===== BRANCH CONTRIBUTIONS =====
branchProps = struct();
branchProps.M  = {'sm',0.70; 'dm',0.30};
branchProps.MP = {'mp',1.00};
branchProps.T  = {'at',0.48; 'mt',0.29; 'pt',0.23};
branchProps.LP = {'ip',0.70; 'sp',0.30};

%% ===== COMPUTE WPCSA / BPCSA FOR LEFT AND RIGHT =====
Wmap = containers.Map('KeyType','char','ValueType','double');
Bmap = containers.Map('KeyType','char','ValueType','double');

% Collect all candidates for final averaging
candidates = struct();

scsKeys = keys(scsMap);

for i = 1:numel(scsKeys)
    key = scsKeys{i};           % e.g. Ma_R, T_L, MP_R, LP_L
    scsVal = scsMap(key);

    [baseMuscle, side] = parseWholeMuscleKey(key);

    if ~isfield(reg, baseMuscle)
        warning('Unknown muscle in SCSA: %s', key);
        continue;
    end

    % Whole-muscle WPCSA and BPCSA
    Wwhole = reg.(baseMuscle).aW * scsVal + reg.(baseMuscle).bW;
    Bwhole = reg.(baseMuscle).aB * scsVal + reg.(baseMuscle).bB;

    % Clamp at zero if needed
    Wwhole = max(Wwhole, 0);
    Bwhole = max(Bwhole, 0);

    % Split to branches
    props = branchProps.(baseMuscle);

    for j = 1:size(props,1)
        branch = lower(props{j,1});
        frac   = props{j,2};

        Wbranch = Wwhole * frac;
        Bbranch = Bwhole * frac;

        outKey = sprintf('%s_%s', branch, lower(side));  % e.g. sm_l
        Wmap(outKey) = Wbranch;
        Bmap(outKey) = Bbranch;

        if ~isfield(candidates, branch)
            candidates.(branch) = struct('value',{},'method',{},'side',{});
        end

        candidates.(branch)(end+1) = struct( ...
            'value',  Wbranch, ...
            'method', 'WPCSA', ...
            'side',   upper(side));

        candidates.(branch)(end+1) = struct( ...
            'value',  Bbranch, ...
            'method', 'BPCSA', ...
            'side',   upper(side));
    end
end

%% ===== WRITE WPCSA.txt =====
writeMapInFixedOrder('WPCSA.txt', Wmap, ...
    {'sm_l','sm_r','dm_l','dm_r','mp_l','mp_r','at_l','at_r','mt_l','mt_r','pt_l','pt_r','ip_l','ip_r','sp_l','sp_r'});

%% ===== WRITE BPCSA.txt =====
writeMapInFixedOrder('BPCSA.txt', Bmap, ...
    {'sm_l','sm_r','dm_l','dm_r','mp_l','mp_r','at_l','at_r','mt_l','mt_r','pt_l','pt_r','ip_l','ip_r','sp_l','sp_r'});

%% ===== FINAL PCSA: average WPCSA/BPCSA candidates, no left/right =====
finalMap = containers.Map('KeyType','char','ValueType','double');
averageLog = {};

targetBranches = {'sm','dm','mp','at','mt','pt','ip','sp'};

for i = 1:numel(targetBranches)
    branch = targetBranches{i};

    if ~isfield(candidates, branch) || isempty(candidates.(branch))
        error('No computed candidates for branch: %s', branch);
    end

    cands = candidates.(branch);
    values = [cands.value];
    finalVal = mean(values);
    finalMap(branch) = finalVal;

    averageLog(end+1,:) = { ...
        branch, ...
        finalVal, ...
        numel(values), ...
        min(values), ...
        max(values)};
end

%% ===== WRITE Final_PCSA.txt =====
writeMapInFixedOrder('Final_PCSA.txt', finalMap, ...
    {'sm','dm','mp','at','mt','pt','ip','sp'});

%% ===== COMPUTE FMAX = Final_PCSA * 40 =====
fmaxMap = containers.Map('KeyType','char','ValueType','double');
finalKeys = keys(finalMap);

for i = 1:numel(finalKeys)
    k = finalKeys{i};
    fmaxMap(k) = finalMap(k) * 40.0;
end

%% ===== WRITE FMAX.txt =====
writeMapInFixedOrder('FMAX.txt', fmaxMap, ...
    {'sm','dm','mp','at','mt','pt','ip','sp'});

%% ===== WRITE AVERAGE LOG =====
fid = fopen('Final_PCSA_average_log.txt','w');
if fid == -1
    error('Could not open Final_PCSA_average_log.txt for writing.');
end

fprintf(fid, 'branch\taverage_pcsa\tn_candidates\tmin_candidate\tmax_candidate\n');
for i = 1:size(averageLog,1)
    fprintf(fid, '%s\t%.6f\t%d\t%.6f\t%.6f\n', ...
        averageLog{i,1}, ...
        averageLog{i,2}, ...
        averageLog{i,3}, ...
        averageLog{i,4}, ...
        averageLog{i,5});
end
fclose(fid);

disp('Done.');
disp('Created files:');
disp('  WPCSA.txt');
disp('  BPCSA.txt');
disp('  Final_PCSA.txt');
disp('  FMAX.txt');
disp('  Final_PCSA_average_log.txt');

%% ============================================================
%% LOCAL FUNCTIONS
%% ============================================================

function scsMap = readSCSAFile(filename)
    % Reads lines like:
    % T_R: 5.437060672411
    % T_L: 5.512818504374
    % Ma_R: 4.982821726731
    % Ma_L: 4.315053432086
    % MP_R: 3.404205178803
    % MP_L: 3.574184289982
    % LP_R: 3.612980890840
    % LP_L: 4.053600469601

    if ~isfile(filename)
        error('File not found: %s', filename);
    end

    fid = fopen(filename,'r');
    if fid == -1
        error('Cannot open file: %s', filename);
    end

    scsMap = containers.Map('KeyType','char','ValueType','double');

    tline = fgetl(fid);
    while ischar(tline)
        line = strtrim(tline);

        if ~isempty(line) && ~startsWith(line,'#') && ~startsWith(line,'%')
            tok = regexp(line, '^([A-Za-z_]+)\s*:?\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', 'tokens', 'once');
            if ~isempty(tok)
                key = strtrim(tok{1});
                val = str2double(tok{2});
                if ~isnan(val)
                    scsMap(key) = val;
                end
            end
        end

        tline = fgetl(fid);
    end

    fclose(fid);
end

function [baseMuscle, side] = parseWholeMuscleKey(key)
    % Accept:
    % Ma_R, Ma_L, M_R, M_L, T_R, T_L, MP_R, MP_L, LP_R, LP_L

    key = strtrim(key);
    key = regexprep(key, ':', '');

    tok = regexp(key, '^(Ma|M|T|MP|LP)_([LR])$', 'tokens', 'once', 'ignorecase');
    if isempty(tok)
        error('Unrecognized SCSA key format: %s', key);
    end

    muscle = upper(tok{1});
    side   = upper(tok{2});

    switch muscle
        case {'MA','M'}
            baseMuscle = 'M';
        case 'T'
            baseMuscle = 'T';
        case 'MP'
            baseMuscle = 'MP';
        case 'LP'
            baseMuscle = 'LP';
        otherwise
            error('Unknown muscle code: %s', muscle);
    end
end

function writeMapInFixedOrder(filename, mapObj, keyOrder)
    fid = fopen(filename,'w');
    if fid == -1
        error('Cannot open %s for writing.', filename);
    end

    for i = 1:numel(keyOrder)
        k = keyOrder{i};
        if isKey(mapObj, k)
            fprintf(fid, '%s\t%.6f\n', k, mapObj(k));
        end
    end

    fclose(fid);
end
