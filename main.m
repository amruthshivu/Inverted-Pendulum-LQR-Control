% ROOT LAUNCHER: Inverted Pendulum LQR Control Project
% Automatically forwards execution to Inverted_Pendulum_LQR/main.m

function main(varargin)
    this_dir = fileparts(mfilename('fullpath'));
    proj_dir = fullfile(this_dir, 'Inverted_Pendulum_LQR');
    addpath(proj_dir);
    addpath(genpath(proj_dir));
    cd(proj_dir);
    if nargin == 0
        feval('main');
    else
        feval('main', varargin{:});
    end
end
