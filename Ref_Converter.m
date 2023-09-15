function [lambda, n] = Ref_Converter(lambda, n_ref )
% lambda or screw data - [um] or [a.u.];  n_ref - raw data from refractometr
if lambda(1) < 100 % it means that someone puts the data from the screw and we need to recalculate them to lambda.
        S_L = @(x) 31431.26866-16939.48903.*x+3523.26502.*x.^2-329.52957.*x.^3+11.63867.*x.^4;
        lambda = S_L(lambda);
end
% P = [ 1.62152, 0.01529, 0.00931, -4.34165e-4, 0.0166];
% n_ref = @(x) 1.34482-1.18718*exp(-x/126.676);

phi = 63;             % is the prism apex angle
N_d = 1.74054;  % The refractive index of the glass prism (Schott SF13) at lambda 589.3[nm] sodium - D line
% N = @( x ) 1.7047+ 0.00715./x.^2+ 0.00169./x.^4 -9.57126e-5./x.^6 +  0.00844.*x.^2; % fit from origin
N = @( x ) 1.70708 + 10943.47279./x.^2 + 2.54416e8./x.^4 + 3.46802e13./x.^6  - 2.93242e-9.*x.^2; % the coefficents were taken from article for SF13 prizm
A = sind(phi) * sqrt( N_d^2 - n_ref.^2 ) - n_ref.*cosd(phi);
n = sind(phi).*sqrt( N( lambda ) .^2 - A.^2) - cosd(phi).*A;


