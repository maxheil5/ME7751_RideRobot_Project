%% validate_fk_kr500_r2830_taskA.m
% ME7751 Project 1 — Task A (FK) validation prep for RoboDK comparison
%
% What this script does:
%  1) Implements FK for KUKA KR 500 R2830 using the Modified DH (DHM) form
%     used by RobotKinematicsCatalogue.
%  2) Generates a set of test joint configurations (deg).
%  3) Prints FK results (full 4x4 T06) and writes them to CSV.
%  4) Provides a spot to paste RoboDK 4x4 pose matrices and computes errors.
%
% Units:
%  - DH lengths in mm -> positions output in mm
%  - Joint angles entered in degrees -> converted to radians internally

clear; clc;

%% ------------------------------------------------------------------------
% 1) Robot definition (KR500 R2830) from RobotKinematicsCatalogue
% DHM columns: [alpha(rad), a(mm), d(mm), theta_offset(rad)]
DHM = [ ...
    0,        0,    1045, 0; ...
   -pi/2,   500,      0, 0; ...
    0,     1300,      0, -pi/2; ...
   -pi/2,   -55,   1025, 0; ...
    pi/2,     0,      0, 0; ...
   -pi/2,     0,    290, pi ...
];

alpha = DHM(:,1);
a     = DHM(:,2);
d     = DHM(:,3);
th0   = DHM(:,4);

% Joint sign convention used by the catalogue (inv_joint)
s = [-1; 1; 1; -1; 1; -1];

% Joint limits from the catalogue (degrees) — optional for random tests
jointMinDeg = [-185, -130, -100, -350, -120, -350];
jointMaxDeg = [ 185,   20,  144,  350,  120,  350];

%% ------------------------------------------------------------------------
% 2) Choose test configurations (degrees)
% IMPORTANT:
% RoboDK "Home" for this model is: [0, -90, 90, 0, 0, 0] deg (from your screenshot).
% We include it as Test 1 to make comparison straightforward.

q_home_RoboDK = [0, -90, 90, 0, 0, 0];
q_zero        = [0,   0,  0, 0, 0, 0];   % still useful as a sanity check

Qdeg_tests = [ ...
    q_home_RoboDK;                      % Test 1: RoboDK Home
    q_zero;                             % Test 2: Zero joint vector
    30,  -20,   40,   10,  -30,   60;   % mixed moderate
   -45,  -60,   20,  -90,   45,  -30;   % wrist turned
    90,  -30,  100,  120,  -60,  180;   % larger but within limits
  -120,   10,  -80, -150,   90, -120;   % near-ish extremes (still safe)
];

% Optional: add random tests (uncomment if desired)
% rng(1);
% nRand = 5;
% marginDeg = 10; % keep away from hard limits
% Qdeg_rand = zeros(nRand,6);
% for k = 1:nRand
%     Qdeg_rand(k,:) = (jointMinDeg+marginDeg) + ...
%         rand(1,6).*( (jointMaxDeg-marginDeg) - (jointMinDeg+marginDeg) );
% end
% Qdeg_tests = [Qdeg_tests; Qdeg_rand];

nTests = size(Qdeg_tests,1);

%% ------------------------------------------------------------------------
% 3) Run FK on each test and store results
results = struct();
results.Qdeg  = Qdeg_tests;
results.T06   = cell(nTests,1);
results.p06   = zeros(nTests,3);
results.R06   = zeros(nTests,9);
results.Tflat = zeros(nTests,16); % row-major flattening of 4x4 for CSV

fprintf('--- KR500 R2830 FK (Modified DHM) ---\n');
for k = 1:nTests
    q_deg = Qdeg_tests(k,:);
    q_rad = deg2rad(q_deg(:));          % column
    theta = th0 + s .* q_rad;           % effective joint angle used in FK

    T06 = eye(4);
    for i = 1:6
        Ti = mdh_T(alpha(i), a(i), d(i), theta(i));
        T06 = T06 * Ti;
    end

    R = T06(1:3,1:3);
    p = T06(1:3,4).';

    results.T06{k}    = T06;
    results.p06(k,:)  = p;
    results.R06(k,:)  = reshape(R,1,9);
    results.Tflat(k,:) = reshape(T06.', 1, 16); % row-major flatten (transpose then reshape)

    % Quick rotation sanity
    rotOrthoErr = norm(R.'*R - eye(3), 'fro');
    detR = det(R);

    fprintf('\nTest %d/%d\n', k, nTests);
    fprintf('q_deg = [%.1f %.1f %.1f %.1f %.1f %.1f]\n', q_deg);
    fprintf('p06 (mm) = [%.3f %.3f %.3f]\n', p);
    fprintf('Rotation check: ||R''R-I||_F = %.3e, det(R) = %.6f\n', rotOrthoErr, detR);

    % Print 4x4 matrix in a copy/paste-friendly format (matches RoboDK style)
    print_matlab_matrix(sprintf('T06_test%d', k), T06);
end

%% ------------------------------------------------------------------------
% 4) Write FK outputs to CSV (for your repo / report)
% (A) Compact CSV: q + p + R
csvHeaderA = ["q1_deg","q2_deg","q3_deg","q4_deg","q5_deg","q6_deg", ...
              "px_mm","py_mm","pz_mm", ...
              "r11","r12","r13","r21","r22","r23","r31","r32","r33"];
csvDataA = [results.Qdeg, results.p06, results.R06];
outCsvA = "taskA_fk_results_kr500_r2830.csv";
write_csv_with_header(outCsvA, csvHeaderA, csvDataA);
fprintf('\nWrote FK results to: %s\n', outCsvA);

% (B) Matrix-shaped CSV: for each test, write 4 rows (one per T06 row)
% Columns: q1..q6, row_index, then the 4 entries of that row of T06.
% In Excel this will visually appear as a 4x4 matrix per test.

outCsvB = "taskA_fk_T06_kr500_r2830.csv";
write_T_pretty_blocks_csv(outCsvB, results.Qdeg, results.T06);
fprintf('Wrote pretty matrix-shaped T06 blocks to: %s\n', outCsvB);

%% ------------------------------------------------------------------------
% 5) Paste RoboDK reference poses here (optional), then compute errors
%
% In RoboDK, set robot joints to q_deg, then copy the pose as a 4x4 matrix (mm)
% and paste each into robodk_T06{k}. Then rerun and get error metrics.

robodk_T06 = cell(nTests,1);

% TEMPLATE (replace with your RoboDK matrices):
% robodk_T06{1} = [ ...
%     1 0 0 0; ...
%     0 1 0 0; ...
%     0 0 1 0; ...
%     0 0 0 1  ...
% ];

haveAll = all(~cellfun(@isempty, robodk_T06));
if haveAll
    fprintf('\n--- Comparing to RoboDK ---\n');
    fprintf('%5s  %12s  %14s\n', 'Test', 'pos_err(mm)', 'ang_err(deg)');
    for k = 1:nTests
        T_fk  = results.T06{k};
        T_ref = robodk_T06{k};

        p_fk  = T_fk(1:3,4);
        p_ref = T_ref(1:3,4);

        R_fk  = T_fk(1:3,1:3);
        R_ref = T_ref(1:3,1:3);

        pos_err = norm(p_fk - p_ref);
        ang_err = rad2deg(rot_angle_err(R_fk, R_ref));

        fprintf('%5d  %12.4f  %14.6f\n', k, pos_err, ang_err);
    end
    fprintf('Done.\n');
else
    fprintf('\nRoboDK reference matrices not provided yet.\n');
    fprintf('Paste robodk_T06{k} matrices and rerun Section 5 for error metrics.\n');
end

%% ===================== Local functions =====================

function T = mdh_T(alpha, a, d, theta)
% mdh_T: Modified DH transform used by RobotKinematicsCatalogue
%
% T = [ c, -s, 0, a;
%       s*ca, c*ca, -sa, -sa*d;
%       s*sa, c*sa,  ca,  ca*d;
%       0, 0, 0, 1 ]

c  = cos(theta); s  = sin(theta);
ca = cos(alpha); sa = sin(alpha);

T = [ ...
    c,      -s,     0,      a; ...
    s*ca,   c*ca,  -sa,   -sa*d; ...
    s*sa,   c*sa,   ca,    ca*d; ...
    0,       0,     0,      1 ...
];
end

function ang = rot_angle_err(R1, R2)
% rot_angle_err: principal angle of relative rotation R_err = R1' * R2
Rerr = R1.' * R2;
val = (trace(Rerr) - 1) / 2;
val = max(-1, min(1, val)); % clamp for numeric stability
ang = acos(val);
end

function print_matlab_matrix(name, M)
% print_matlab_matrix: prints a matrix in copy/paste MATLAB form
fprintf('%s = [\n', name);
for r = 1:size(M,1)
    fprintf('  %.6f  %.6f  %.6f  %.6f;\n', M(r,1), M(r,2), M(r,3), M(r,4));
end
fprintf('];\n');
end

function write_csv_with_header(filename, headerStrings, data)
% write_csv_with_header: writes CSV with a header row (no toolboxes required)
fid = fopen(filename, 'w');
assert(fid ~= -1, 'Could not open %s for writing.', filename);

% header
for i = 1:numel(headerStrings)
    if i < numel(headerStrings)
        fprintf(fid, '%s,', headerStrings(i));
    else
        fprintf(fid, '%s\n', headerStrings(i));
    end
end

% data
fmtRow = [repmat('%.10g,', 1, size(data,2)-1), '%.10g\n'];
for r = 1:size(data,1)
    fprintf(fid, fmtRow, data(r,:));
end

fclose(fid);
end

function write_T_pretty_blocks_csv(filename, Qdeg, Tcells)
% Writes a "pretty" CSV for Excel:
%  - One row with q values (once)
%  - Then 4 rows containing the 4x4 T matrix
%  - Then one blank row separator
%
% Layout:
%   Row A:  q1 q2 q3 q4 q5 q6 | (blank columns)
%   Rows B-E:             | T11 T12 T13 T14
%                         | T21 T22 T23 T24
%                         | T31 T32 T33 T34
%                         | T41 T42 T43 T44
%   Row F: blank

fid = fopen(filename, 'w');
assert(fid ~= -1, 'Could not open %s for writing.', filename);

% Header row (keep simple and readable)
fprintf(fid, 'q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg,,t1,t2,t3,t4\n');

nTests = size(Qdeg,1);
for k = 1:nTests
    q = Qdeg(k,:);
    T = Tcells{k};

    % Row 1: q values only (matrix columns blank)
    fprintf(fid, '%.10g,%.10g,%.10g,%.10g,%.10g,%.10g,,,,,\n', q(1),q(2),q(3),q(4),q(5),q(6));

    % Rows 2-5: the 4x4 matrix
    for r = 1:4
        fprintf(fid, ',,,,,,,%.10g,%.10g,%.10g,%.10g\n', T(r,1), T(r,2), T(r,3), T(r,4));
    end

    % Blank separator row
    fprintf(fid, ',,,,,,,,,,\n');
end

fclose(fid);
end
