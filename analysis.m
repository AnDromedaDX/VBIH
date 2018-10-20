Fsam = 1500;
Fnyq = Fsam/2;




fe = 15;
fb = 5;

freq = 10.0;

[b,a]=butter(3, [fb/Fnyq, fe/Fnyq]);


inp = load('C:/Users/Andriy/Desktop/octave/signal_3.mat');
input = inp.input;

output=filter(b,a,input);



n = length(input);
X = fft(input);
f = (0:n-1)*(Fsam/n);     %frequency range
power = abs(X).^2/n;      %power

Y = fftshift(X);
fshift = (-n/2:n/2-1)*(Fsam/n); % zero-centered frequency range
powershift = abs(Y).^2/n;     % zero-centered power

O = fft(output);
shiftO = fftshift(O);
powershiftO = abs(shiftO).^2/n; 

for i = fshift
   idx = find(fshift == i);
   if powershiftO(idx) > 100
      if i > 0
        if abs(freq - i) < 1.0
         system('python C:/Users/Andriy/Desktop/octave/tcp_client_octave.py Stop') 
         disp(i);
         disp(powershiftO(idx));
        endif
       endif
   endif
endfor

   


clf
subplot ( 4, 1, 1);
plot(t, [input]);

subplot ( 4, 1, 2);
plot(t, [output]);

subplot ( 4, 1, 3 );
plot(fshift,powershift);

subplot ( 4, 1, 4 );
plot(fshift,powershiftO);