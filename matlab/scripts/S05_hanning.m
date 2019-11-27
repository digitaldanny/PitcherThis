% This script generates the coefficients for a C hanning window LUT to 
% decrease the processing time spent on windowing.

L = 256; 

coef = hann(L);
str1 = sprintf('const float hanningLUT[%s] = {\n', string(L));
str2 = sprintf('\t\t%ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff, \n', coef); 
str3 = sprintf('};\n');
str = sprintf('%s%s%s', str1, str2, str3)