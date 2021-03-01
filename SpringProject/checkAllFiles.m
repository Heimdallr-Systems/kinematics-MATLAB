function checkAllFiles(useModCyc)
% Run checkcode against all files in the current directory
% Any value provided as an argument will cause the program
% to also output cyclomatic complexity.
files = dir(".");

narginchk(0, 1)

for index=1:numel(files)
    path = strcat(files(index).folder, "\", files(index).name);
    disp(path)
    if endsWith(path, ".m") && ~files(index).isdir
    if nargin==1
        checkcode(path, "-fillpath", "-modcyc");
    else
        checkcode(path, "-fullpath");
    end
    end
end