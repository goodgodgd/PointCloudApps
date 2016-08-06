clc
clear

s1.name='alice';
s1.age=22;
s1.height=175;
s1
s2.name='alex';
s2.age=23;
s2.height=180;
s2

fields = fieldnames(s1)';
fields(2,:) = cellfun(@(f) [s1.(f) s2.(f)], fields, 'unif', false);
smerge = struct(fields{:})
