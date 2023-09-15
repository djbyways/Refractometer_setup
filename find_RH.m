function RH = find_RH(Vout,T,sensor)

% calibration @Vin = 5 V, T in [C]
% RH in %
Vin = 5.0; % voltage of power supply

switch sensor

  case '210' %check - JA was using?
	offset = 0.826;
    slope = 31.250e-3;
	
  case '212' %is very off
    offset = 0.826;
    slope = 31.449e-3;
    
  case '213' %is dead
    offset = 0.819;
    slope = 31.164e-3;
    
  case '33' %HIH-4000 3.5% accuracy
    offset = 0.733967;
    slope = 30.323022e-3;  
    
end
        
RH = ((Vout*5/Vin - offset)/slope)/(1.0546-0.00216*T);