clc
clear

for i=1:10
    try
        error('error msg %d', i);
    catch ME
        ME.message
    end
end