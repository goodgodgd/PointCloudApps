clc
clear

field = 'f';
value = {'some text';
         [10, 20, 30];
         magic(5)};
a = struct(field,value)

field1 = 'f1';  value1 = zeros(1,10);
field2 = 'f2';  value2 = {'a', 'b'};
field3 = 'f3';  value3 = {pi, pi.^2};
field4 = 'f4';  value4 = {'fourth'};
b = struct(field1,value1,field2,value2,field3,value3,field4,value4)

c = struct;
c.f = 1;
d(1) = c;
d(5) = c;