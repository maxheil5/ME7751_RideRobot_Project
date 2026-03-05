%% taskC_verify_IK_KR500_R2830.m
% ME 7751 Project 1 — Task C: Thorough IK Verification (Forward-then-Inverse)
%
% Procedure:
%   1) Choose test joint configurations q_test (include near singularities & limits).
%   2) Compute T_test = FK(q_test).
%   3) Run IK(T_test) to obtain all solution branches q_sol^(k).
%   4) For each solution, verify FK(q_sol^(k)) ≈ T_test (within tolerance).
%   5) Cross-validate with RoboDK for selected poses (manual step):
%      - Use the printed T_test matrices and compare RoboDK joint solutions
%        to your analytical IK branches. Save screenshots/numeric comparisons.
%   6) Report results in clear tables, noting singular/degenerate cases.
%
% Outputs:
%   - taskC_summary.csv   : one row per test pose
%   - taskC_detail.csv    : one row per IK solution per test pose
%   - taskC_T06_pretty_blocks.csv : "nice" CSV blocks: q row + 4x4 T + blank line
%
% Requirements:
%   - IK_KR500_R2830.m must be on the MATLAB path
%   - This script contains its own FK implementation (Modified DHM consistent with Task A)
%
% Units:
%   - FK uses mm for positions, radians for joint angles.
%   - Test inputs below are in degrees and converted to radians.

clear; clc;

%% ------------------------- Settings -------------------------
posTol_mm   = 1e-6;     % FK/IK consistency tolerance on position (mm)
angTol_deg  = 1e-8;     % FK/IK consistency tolerance on orientation (deg)
eps_wrist   = 1e-8;     % wrist singular threshold (matches IK default)

% Joint limits (deg) for KR500 R2830
jminDeg = [-185, -130, -100, -350, -120, -350];
jmaxDeg = [ 185,   20,  144,  350,  120,  350];

% Handy near-limit margin (deg)
m = 1;  % 1 degree inside hard limit

%% ------------------------- Test Set (degrees) -------------------------
% Include: RoboDK Home, zero, moderate randoms, near wrist singularities (q5≈0),
% and near joint limits.

q_home = [0, -90,  90,   0,   0,   0];      % RoboDK Home
q_zero = [0,   0,   0,   0,   0,   0];

Qdeg_tests = [
    q_home;
    q_zero;

    % Moderate mixed tests (safe)
     30,  -20,   40,   10,  -30,   60;
    -45,  -60,   20,  -90,   45,  -30;

    % Wrist near-singular: q5 ~ 0 (two different wrist orientations)
     20,  -40,   60,   90,    0.01,  45;
    -60,  -30,   80, -120,   -0.01, -90;

    % Near joint limits (inside by margin m)
    (jmaxDeg(1)-m),  -60,   40,   0,   10,    0;   % q1 near max
    (jminDeg(1)+m),  -60,   40,   0,   10,    0;   % q1 near min

      0, (jmaxDeg(2)-m),   20,   0,   10,    0;    % q2 near max
      0, (jminDeg(2)+m),   20,   0,   10,    0;    % q2 near min

      0,  -60, (jmaxDeg(3)-m),   0,   10,    0;    % q3 near max
      0,  -60, (jminDeg(3)+m),   0,   10,    0;    % q3 near min

      0,  -60,   40, (jmaxDeg(4)-m),  10,    0;    % q4 near max
      0,  -60,   40, (jminDeg(4)+m),  10,    0;    % q4 near min

      0,  -60,   40,   0, (jmaxDeg(5)-m),    0;    % q5 near max
      0,  -60,   40,   0, (jminDeg(5)+m),    0;    % q5 near min

      0,  -60,   40,   0,   10, (jmaxDeg(6)-m);    % q6 near max
      0,  -60,   40,   0,   10, (jminDeg(6)+m);    % q6 near min
];

nTests = size(Qdeg_tests,1);

%% ------------------------- Storage -------------------------
summaryRows = [];  % per test
detailRows  = [];  % per solution

% For "pretty" T blocks CSV
Tcells = cell(nTests,1);

fprintf('=== Task C — IK Verification: KR500 R2830 ===\n');
fprintf('Tests: %d\n\n', nTests);

%% ------------------------- Main Loop -------------------------
for t = 1:nTests
    qTest_deg = Qdeg_tests(t,:);
    qTest_rad = deg2rad(qTest_deg);

    % 2) FK: T_test = FK(q_test)
    T_test = FK_KR500_R2830(qTest_rad);
    Tcells{t} = T_test;

    % 3) IK: all branches
    opts = struct();
    opts.eps_wrist  = eps_wrist;
    opts.seed       = qTest_rad;  % sort near the original configuration
    [Qsol, infoIK] = IK_KR500_R2830(T_test, opts); %#ok<NASGU>

    nSol = size(Qsol,1);

    % If no solutions, still record summary
    if nSol == 0
        summaryRows = [summaryRows; t, qTest_deg, 0, NaN, NaN, 0, 0]; %#ok<AGROW>
        fprintf('Test %2d: q_deg=[%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f] -> IK returned 0 solutions\n', ...
            t, qTest_deg);
        continue;
    end

    % 4) For each solution, verify FK(q_sol) ≈ T_test
    posErrs = zeros(nSol,1);
    angErrs = zeros(nSol,1);
    dqNorm  = zeros(nSol,1);
    singFlg = zeros(nSol,1);

    recovered = false;

    for k = 1:nSol
        qk = Qsol(k,:);
        T_fk = FK_KR500_R2830(qk);

        % position error (mm)
        posErr = norm(T_fk(1:3,4) - T_test(1:3,4));

        % orientation error (deg) via relative rotation angle
        Rerr = T_fk(1:3,1:3).' * T_test(1:3,1:3);
        angErr = rad2deg(acos(clamp((trace(Rerr)-1)/2)));

        % distance to original (deg), accounting wrap
        dq = wrapToPiVec(qk - qTest_rad);
        dqNorm_deg = rad2deg(norm(dq));

        % flag wrist singular (based on this candidate)
        singFlg(k) = is_wrist_singular(qk, T_test(1:3,1:3), eps_wrist);

        posErrs(k) = posErr;
        angErrs(k) = angErr;
        dqNorm(k)  = dqNorm_deg;

        % recovered if within small joint-distance OR within strict FK tolerance
        if dqNorm_deg < 1e-5 || (posErr < posTol_mm && angErr < angTol_deg && dqNorm_deg < 1e-2)
            recovered = true;
        end

        % detail row:
        % [test_id, sol_id, qtest(6), qsol(6), posErr, angErr, dqNorm, singFlag]
        detailRows = [detailRows; ...
            t, k, qTest_deg, rad2deg(qk), posErr, angErr, dqNorm_deg, singFlg(k)]; %#ok<AGROW>
    end

    % Summary metrics for this test
    maxPos = max(posErrs);
    maxAng = max(angErrs);
    nPass  = sum(posErrs < posTol_mm & angErrs < angTol_deg);
    nSing  = sum(singFlg ~= 0);

    summaryRows = [summaryRows; t, qTest_deg, nSol, maxPos, maxAng, recovered, nSing]; %#ok<AGROW>

    fprintf('Test %2d: q_deg=[%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f] | sols=%d | pass=%d | maxPos=%.3e mm | maxAng=%.3e deg | recovered=%d | wristSingFlags=%d\n', ...
        t, qTest_deg, nSol, nPass, maxPos, maxAng, recovered, nSing);
end

fprintf('\nDone.\n');

%% ------------------------- Write CSV outputs -------------------------

% Summary CSV
summaryHeader = [ ...
    "test_id","q1_deg","q2_deg","q3_deg","q4_deg","q5_deg","q6_deg", ...
    "n_solutions","max_pos_err_mm","max_ang_err_deg","recovered_original","n_wrist_singular_flags" ...
];
summaryCsv = "taskC_summary.csv";
write_csv_with_header(summaryCsv, summaryHeader, summaryRows);
fprintf('Wrote: %s\n', summaryCsv);

% Detail CSV
detailHeader = [ ...
    "test_id","sol_id", ...
    "q1_test_deg","q2_test_deg","q3_test_deg","q4_test_deg","q5_test_deg","q6_test_deg", ...
    "q1_sol_deg","q2_sol_deg","q3_sol_deg","q4_sol_deg","q5_sol_deg","q6_sol_deg", ...
    "pos_err_mm","ang_err_deg","dq_norm_deg","wrist_singular_flag" ...
];
detailCsv = "taskC_detail.csv";
write_csv_with_header(detailCsv, detailHeader, detailRows);
fprintf('Wrote: %s\n', detailCsv);

% Pretty T blocks CSV for RoboDK cross-validation convenience
prettyCsv = "taskC_T06_pretty_blocks.csv";
write_T_pretty_blocks_csv(prettyCsv, Qdeg_tests, Tcells);
fprintf('Wrote: %s\n', prettyCsv);

%% =====================================================================
% Local functions
% =====================================================================

function T06 = FK_KR500_R2830(q)
% FK for KR500 R2830 using Modified DH (DHM) and the same sign/offset convention as Task A.
% Input q: 1x6 radians
% Output T06: 4x4, mm

q = q(:);

% DHM: [alpha, a, d, theta_offset]
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

% inv_joint sign vector
s = [-1; 1; 1; -1; 1; -1];

theta = th0 + s.*q;

T06 = eye(4);
for i = 1:6
    T06 = T06 * mdh_T(alpha(i), a(i), d(i), theta(i));
end
end

function T = mdh_T(alpha, a, d, theta)
% Modified DH transform used by RobotKinematicsCatalogue
c  = cos(theta); s  = sin(theta);
ca = cos(alpha); sa = sin(alpha);

T = [ ...
    c,      -s,     0,      a; ...
    s*ca,   c*ca,  -sa,   -sa*d; ...
    s*sa,   c*sa,   ca,    ca*d; ...
    0,       0,     0,      1 ...
];
end

function flag = is_wrist_singular(q, R06, eps_wrist)
% Flag wrist singularity based on s5 computed from R36:
% s5 = sqrt(R36(2,1)^2 + R36(2,2)^2). If s5 <= eps => singular.
q = q(:);
q1 = q(1); q2 = q(2); q3 = q(3);

c1 = cos(q1); s1 = sin(q1);
c23 = cos(q2+q3); s23 = sin(q2+q3);

R03 = [ s23*c1,   c23*c1,   s1;
       -s23*s1,  -c23*s1,   c1;
        c23,     -s23,      0 ];

R36 = R03.' * R06;
s5 = hypot(R36(2,1), R36(2,2));
flag = (s5 <= eps_wrist);
end

function v = clamp(v)
v = max(-1, min(1, v));
end

function a = wrapToPiVec(a)
a = mod(a + pi, 2*pi) - pi;
end

function write_csv_with_header(filename, headerStrings, data)
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
% Pretty CSV for Excel:
%   - One row containing q values (deg)
%   - Next 4 rows are the 4x4 T matrix block
%   - One blank row separator
fid = fopen(filename, 'w');
assert(fid ~= -1, 'Could not open %s for writing.', filename);

fprintf(fid, 'q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg,,t1,t2,t3,t4\n');

nTests = size(Qdeg,1);
for k = 1:nTests
    q = Qdeg(k,:);
    T = Tcells{k};

    % Row: q only
    fprintf(fid, '%.10g,%.10g,%.10g,%.10g,%.10g,%.10g,,,,,\n', q(1),q(2),q(3),q(4),q(5),q(6));

    % Rows: 4x4 T
    for r = 1:4
        fprintf(fid, ',,,,,,,%.10g,%.10g,%.10g,%.10g\n', T(r,1), T(r,2), T(r,3), T(r,4));
    end

    % Blank separator
    fprintf(fid, ',,,,,,,,,,\n');
end

fclose(fid);
end