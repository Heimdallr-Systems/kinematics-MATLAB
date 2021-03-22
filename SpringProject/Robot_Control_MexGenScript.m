% ROBOT_CONTROL_MEXGENSCRIPT   Generate MEX-function Robot_Control_mex from
%  Robot_Control.
% 
% Script generated from project 'Robot_Control.prj' on 15-Mar-2021.
% 
% See also CODER, CODER.CONFIG, CODER.TYPEOF, CODEGEN.

%% Create configuration object of class 'coder.MexCodeConfig'.
cfg = coder.config('mex');
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'Codegen';
cfg.GenerateReport = true;
cfg.ReportPotentialDifferences = false;
cfg.EnableRuntimeRecursion = false;
cfg.DynamicMemoryAllocation = 'Off';
cfg.InlineBetweenUserFunctions = 'Never';
cfg.InlineBetweenUserAndMathWorksFunctions = 'Never';
cfg.InlineBetweenMathWorksFunctions = 'Never';
cfg.MATLABSourceComments = true;
cfg.EnableDebugging = true;

%% Define argument types for entry-point 'Robot_Control'.
ARGS = cell(1,1);
ARGS{1} = cell(4,1);
ARGS{1}{1} = coder.typeof(0,[3 1]);
ARGS{1}{2} = coder.typeof(0,[1 3]);
ARGS{1}{3} = coder.typeof(0,[36  1]);
ARGS{1}{4} = coder.typeof(0);

%% Invoke MATLAB Coder.
codegen -config cfg Robot_Control -args ARGS{1}

