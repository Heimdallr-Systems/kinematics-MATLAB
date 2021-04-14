function [indices] = findTrue4Elem(arrayToSearch)
% Searces through a four element array and returns the indices to the
% elements that are true. Returned array is always 4 elements even if less
% than four elements are true. This allows fixed length code generation. 
indices = uint8(zeros(1,4));

elToAppend = uint8(1);

for elToChk=uint8(1:4)
    if arrayToSearch(elToChk)==true
        indices(elToAppend) = elToChk;
        elToAppend = elToAppend + 1;
    end
end

