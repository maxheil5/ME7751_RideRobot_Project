%% IK_KR500_R2830.m
% Analytical IK for KUKA KR 500 R2830 (Modified DHM convention)
% - Enumerates up to 8 branches: shoulder (2) x elbow (2) x wrist (2)
% - Handles wrist singularities (s5 ~ 0)
% - Filters by joint limits
% - Returns all valid solutions and diagnostic info
%
% Usage:
%   [Q, info] = IK_KR500_R2830(T06);
%   % Q: Mx6 solutions (radians)
%   % info: struct with branch bookkeeping
%
% Notes:
% - T06 must be 4x4 homogeneous transform with position in mm.
% - Solutions are returned in radians (consistent with FK code).
% - Joint limits are taken from the catalogue (deg) and applied after wrapping.
%
function [Q, info] = IK_KR500_R2830(T06, opts)

if nargin < 2, opts = struct(); end
opts = fill_defaults(opts);

assert(all(size(T06)==[4 4]), 'T06 must be 4x4.');
R06 = T06(1:3,1:3);
p06 = T06(1:3,4);

% ---- Robot constants (from RobotKinematicsCatalogue KR500 R2830) ----
d6 = 290;   % mm
A  = 1300;  % mm (link-1 planar length)
d1 = 1045;  % mm

% link-2 effective planar length from (1025, 55) pair:
L = hypot(1025, 55);              % sqrt(1025^2 + 55^2)
phi = atan2(55, 1025);            % rad

% ---- Joint limits (deg -> rad) ----
jmin = deg2rad([-185, -130, -100, -350, -120, -350]);
jmax = deg2rad([ 185,   20,  144,  350,  120,  350]);

% ---- Wrist center ----
z6 = R06(:,3);
p_wc = p06 - d6*z6;  % mm
xw = p_wc(1); yw = p_wc(2); zw = p_wc(3);

rho = hypot(xw, yw);

% Shoulder branches: (q1, B)
q1_a = atan2(-yw, xw);
B_a  = +rho;

q1_b = wrapToPi(q1_a + pi);
B_b  = -rho;

shoulders = [q1_a, B_a; q1_b, B_b];

% Preallocate outputs
Q_all = [];
branch_all = [];  % [S E W] indices
status_all = [];

% Iterate shoulder branches
for sIdx = 1:2
    q1 = shoulders(sIdx,1);
    B  = shoulders(sIdx,2);

    % Planar reduction
    r = B - 500;        % mm
    z = d1 - zw;        % mm

    % Elbow feasibility (law of cosines)
    D = (r^2 + z^2 - A^2 - L^2) / (2*A*L);

    if abs(D) > 1 + opts.tol_reach
        % unreachable for this shoulder branch
        continue;
    end
    D = max(-1, min(1, D));  % clamp for numeric stability

    % Two elbow branches (gamma)
    gamma_plus  = atan2(+sqrt(max(0,1-D^2)), D);
    gamma_minus = atan2(-sqrt(max(0,1-D^2)), D);

    gammas = [gamma_plus; gamma_minus];   % E=1 -> +, E=2 -> -

    for eIdx = 1:2
        gamma = gammas(eIdx);

        % q2 and q3
        q2 = atan2(z, r) - atan2(L*sin(gamma), A + L*cos(gamma));
        q3 = gamma - phi;

        q1n = wrapToPi(q1);
        q2n = wrapToPi(q2);
        q3n = wrapToPi(q3);

        % Compute R03 (closed-form from Task A)
        c1 = cos(q1n); s1 = sin(q1n);
        c23 = cos(q2n+q3n); s23 = sin(q2n+q3n);
        R03 = [ s23*c1,   c23*c1,   s1;
               -s23*s1,  -c23*s1,   c1;
                c23,     -s23,      0 ];

        R36 = R03.' * R06;

        % Wrist extraction
        c5 = R36(2,3);
        s5 = hypot(R36(2,1), R36(2,2));  % sqrt(r21^2 + r22^2)

        if s5 > opts.eps_wrist
            % ----- Non-singular: two wrist branches -----
            th5a = atan2(+s5, c5);
            th4a = atan2(R36(3,3), -R36(1,3));
            th6a = atan2(-R36(2,2), R36(2,1));

            % Branch W1 (no flip): (th4a, th5a, th6a)
            [q4_1, q5_1, q6_1] = map_theta_to_q(th4a, th5a, th6a);
            qW1 = [q1n, q2n, q3n, q4_1, q5_1, q6_1];

            % Branch W2 (flip): th5=-th5, th4+=pi, th6+=pi
            th5b = -th5a;
            th4b = wrapToPi(th4a + pi);
            th6b = wrapToPi(th6a + pi);

            [q4_2, q5_2, q6_2] = map_theta_to_q(th4b, th5b, th6b);
            qW2 = [q1n, q2n, q3n, q4_2, q5_2, q6_2];

            % Normalize/wrap and limit-check both
            [qW1_ok, qW1_adj] = enforce_limits(qW1, jmin, jmax);
            [qW2_ok, qW2_adj] = enforce_limits(qW2, jmin, jmax);

            if qW1_ok
                Q_all = [Q_all; qW1_adj]; %#ok<AGROW>
                branch_all = [branch_all; sIdx, eIdx, 1]; %#ok<AGROW>
                status_all = [status_all; 0]; %#ok<AGROW>
            end
            if qW2_ok
                Q_all = [Q_all; qW2_adj]; %#ok<AGROW>
                branch_all = [branch_all; sIdx, eIdx, 2]; %#ok<AGROW>
                status_all = [status_all; 0]; %#ok<AGROW>
            end

        else
            % ----- Wrist singular: only coupled DOF available -----
            % Here theta5 is ~0 or ~pi. We'll choose theta4=0 and solve theta6 from psi.
            % This produces a consistent solution; there is an infinite family otherwise.

            % Determine if theta5 ~ 0 (c5 ~ +1) or theta5 ~ pi (c5 ~ -1)
            if c5 >= 0
                % theta5 ~ 0
                th5 = 0;
                psi = atan2(-R36(1,2), R36(1,1));  % theta4 + theta6
                th4 = 0;
                th6 = psi;
            else
                % theta5 ~ pi
                th5 = pi;
                psi = atan2(-R36(1,2), -R36(1,1)); % theta4 - theta6
                th4 = 0;
                th6 = -psi;
            end

            [q4, q5, q6] = map_theta_to_q(th4, th5, th6);
            qSing = [q1n, q2n, q3n, q4, q5, q6];

            [ok, qAdj] = enforce_limits(qSing, jmin, jmax);
            if ok
                Q_all = [Q_all; qAdj]; %#ok<AGROW>
                branch_all = [branch_all; sIdx, eIdx, 0]; %#ok<AGROW> % W=0 indicates singular-handled
                status_all = [status_all; 1]; %#ok<AGROW> % 1 => singular used
            end
        end
    end
end

% Remove duplicates (common when wraps/branches coincide)
Q = unique_rows_wrap(Q_all, opts.tol_unique);

% Optional: sort by closeness to seed
if ~isempty(opts.seed)
    seed = opts.seed(:).';
    d = sum(angle_diff(Q, seed).^2, 2);
    [~, idx] = sort(d, 'ascend');
    Q = Q(idx,:);
end

% Info struct
info = struct();
info.p_wc = p_wc;
info.branches = branch_all;
info.status = status_all;
info.jointMin = jmin;
info.jointMax = jmax;
info.opts = opts;

end

%% ======================= Helpers =======================

function opts = fill_defaults(opts)
if ~isfield(opts,'eps_wrist'),  opts.eps_wrist = 1e-8; end
if ~isfield(opts,'tol_reach'),  opts.tol_reach = 1e-10; end
if ~isfield(opts,'tol_unique'), opts.tol_unique = 1e-6; end
if ~isfield(opts,'seed'),       opts.seed = []; end
end

function [q4,q5,q6] = map_theta_to_q(th4, th5, th6)
% Mapping from theta-space (used in per-link DH) to RoboDK joint angles q
% Given:
%   theta4 = -q4
%   theta5 =  q5
%   theta6 =  pi - q6
q4 = wrapToPi(-th4);
q5 = wrapToPi(th5);
q6 = wrapToPi(pi - th6);
end

function [ok, qAdj] = enforce_limits(q, jmin, jmax)
% Try to wrap each joint by adding/subtracting 2*pi to land within limits.
% If multiple wraps possible, choose the one closest to the original q.
qAdj = q;

for i = 1:6
    qi = qAdj(i);

    % generate candidate wraps
    cand = [qi, qi+2*pi, qi-2*pi, qi+4*pi, qi-4*pi];
    % choose those within limits
    in = cand(cand >= jmin(i)-1e-12 & cand <= jmax(i)+1e-12);

    if isempty(in)
        ok = false;
        return;
    end

    % pick candidate closest to original qi
    [~,k] = min(abs(in - qi));
    qAdj(i) = in(k);
end

ok = true;
end

function Quniq = unique_rows_wrap(Q, tol)
% Removes near-duplicate solutions (accounting for angle wrap).
if isempty(Q)
    Quniq = Q;
    return;
end

Quniq = Q(1,:);
for i = 2:size(Q,1)
    qi = Q(i,:);
    isDup = false;
    for j = 1:size(Quniq,1)
        if all(abs(wrapToPi(qi - Quniq(j,:))) < tol)
            isDup = true;
            break;
        end
    end
    if ~isDup
        Quniq = [Quniq; qi]; %#ok<AGROW>
    end
end
end

function d = angle_diff(Q, seed)
% elementwise wrapped difference
d = wrapToPi(Q - seed);
end

function ang = wrapToPi(ang)
% local wrapToPi to avoid toolbox dependency
ang = mod(ang + pi, 2*pi) - pi;
end