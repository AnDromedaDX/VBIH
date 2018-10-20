Fsam = 1500;
Fnyq = Fsam/2;

% Frequencies
f1 = 10;
f2 = 50;
f3 = 120;
f4 = 57;
f5 = 222;
f6 = 78;

df = 5;

[b,a]=butter(2, [(f1-df)/Fnyq, (f1+df)/Fnyq]);


%[H,w] = freqz(b,a);        % Compute its frequency response

% Plot the frequency response H(w):
%
%figure(1);
%freqz(b,a,1024)

%freqplot(w,abs(H),'-k','Amplitude Response',...
%         'Frequency (rad/sample)', 'Gain');
%saveplot('../eps/freqzDemoOne.eps');
%pause;



t=0:1/Fsam:5;

%inp = [];

inp = zeros(6, length(t));



inp([1],:)=sin(2*pi*f1*t) + cos(2*pi*f2*t) + randn(size(t));
inp([2],:)=cos(2*pi*f3*t) + sin(2*pi*f2*t) + randn(size(t));
inp([3],:)=sin(2*pi*f1*t) + cos(2*pi*f5*t) + randn(size(t));
inp([4],:)=sin(2*pi*f6*t) + sin(2*pi*f4*t) + randn(size(t));
inp([5],:)=cos(2*pi*f2*t) + sin(2*pi*f3*t) + randn(size(t));
inp([6],:)=cos(2*pi*f4*t) + cos(2*pi*f5*t) + randn(size(t));

for i = 1:6
  input = inp([i],:);
  save( ['signal_' num2str(i) '.mat'], 'input');
endfor


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

clf
subplot ( 10, 1, 1);
plot(t, [inp([1],:)]);

subplot ( 10, 1, 2);
plot(t, [input1]);

subplot ( 10, 1, 3);
plot(t, [input2]);

subplot ( 10, 1, 4);
plot(t, [input3]);

subplot ( 10, 1, 5);
plot(t, [input4]);

subplot ( 10, 1, 6);
plot(t, [input5]);

subplot ( 10, 1, 7);
plot(t, [input6]);

subplot ( 10, 1, 8 );
plot(f,power);

subplot ( 10, 1, 9 );
plot(fshift,powershift);

subplot ( 10, 1, 10 );
plot(fshift,powershiftO);
