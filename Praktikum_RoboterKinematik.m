%% Aufgabe 1

%% Definition symbolischer Variablen

clear; close; clc;

% Symbolische Variablen definieren 
syms alpha beta gamma delta epsilon phi L1 L2 L3 L4

%% Aufstellen der einzelnen homogenen Transformationsmatrizen

% Koordinatensystem 0 ZU 1
% Rotation um die y0-Achse um Winkel alpha (entgegen den Uhrzeigersinn)
S_01 = [cos(alpha), 0, sin(alpha), 0; 
              0, 1, 0, 0; 
              -sin(alpha), 0, cos(alpha), 0; 
              0, 0, 0, 1];
          
% KOORDINATENSYSTEM 1 ZU 2
% Translation entlang der y1-Achse um L1
T_12 = [1, 0, 0, 0;
               0, 1, 0, L1;
               0, 0, 1, 0;
               0, 0, 0, 1];
% Rotation um verschobene z1-Achse um Winkel beta (entgegen den Uhrzeigersinn)
S_12 = [cos(beta),  -sin(beta), 0, 0; 
              sin(beta), cos(beta), 0, 0; 
              0, 0, 1, 0;
              0, 0, 0, 1];
          
% KOORDINATENSYSTEM 2 ZU 3
% Translation entlang der x2-Achse um L2
T_23 = [1, 0, 0, L2;
               0, 1, 0, 0;
               0, 0, 1, 0;
               0, 0, 0, 1];      
% Rotation um verschobene z2-Achse um Winkel gamma (im Uhrzeigersinn) 
S_23 = [cos(gamma),  sin(gamma), 0, 0; 
              -sin(gamma), cos(gamma), 0, 0; 
              0, 0, 1, 0;
              0, 0, 0, 1];

% KOORDINATENSYSTEM 3 ZU 4
% Translation entlang der x3-Achse um L3
T_34 = [1, 0, 0, L3;
               0, 1, 0, 0;
               0, 0, 1, 0;
               0, 0, 0, 1];      
% Rotation um verschobene x3-Achse um Winkel delta (entgegen den Uhrzeigersinn) 
S_34 = [1, 0, 0, 0
              0, cos(delta), -sin(delta), 0; 
              0, sin(delta), cos(delta), 0;
              0, 0, 0, 1];
          
% KOORDINATEN SYSTEM 5
% Rotation um die y4-Achse um Winkel epsilon (entgegen den Uhrzeigersinn)
S_45 = [cos(epsilon), 0, sin(epsilon), 0; 
              0, 1, 0, 0; 
              -sin(epsilon), 0, cos(epsilon), 0; 
              0, 0, 0, 1];
          
% KOORDINATEN SYSTEM 6
% Rotation um die z5-Achse um Winkel phi (entgegen den Uhrzeigersinn)
S_56 = [cos(phi),  -sin(phi), 0, 0; 
              sin(phi), cos(phi), 0, 0; 
              0, 0, 1, 0;
              0, 0, 0, 1];
% Translation entlang der z6-Achse um L4
T_67 = [1, 0, 0, 0;
               0, 1, 0, 0;
               0, 0, 1, L4;
               0, 0, 0, 1];  
                     
%% Berechnung von H zur Ermittlung des Ortsvektors r_0 und der Drehmatrix S_06

% H berechnen durch Multiplikation der Transformationsmatrizen
H = S_01 * T_12 * S_12 * T_23 * S_23 * T_34 * S_34 * S_45 * S_56 * T_67;

% Ortsvektor der Greiferspitze r_0 (Endpunkt von L4 im Intertialsystem) aus H entnehmen
r_0 = H(1:3, 4);

% Drehmatrix S_06 aus H entnehmen
S_06 = H(1:3, 1:3);

%% Überprüfung der Determinante von S_06

% Bestimmung der Determinante (Die Determinante von Drehmatrizen = 1)
det_S_06 = simplify(det(S_06));
% Rückmeldung im Worspace anzeigen
if det_S_06 == 1 
    fprintf('Determinante ist gleich 1 :-) \n');
else 
    fprintf('Determinante ist ungleich 1 :-( \n');
end


% ******************************************************************************
%% Aufgabe 2

% --------------------------------------------------------------------
%%      INPUT

%% Vorgegebene Parameter eingeben

% Ballaufnahme (B) Position und Orientierung
r_0_B = [200; 20; 250]; % [mm]
S_06_B = [sqrt(2)/2, sqrt(2)/2, 0; 0, 0, -1; -(sqrt(2)/2) sqrt(2)/2 0];
S_06_B_rot = [sqrt(2)/2, sqrt(2)/2, 0, 0; 0, 0, -1, 0; -(sqrt(2)/2) sqrt(2)/2 0, 0; 0, 0, 0, 1]; 

% Korb (K) Position und Orientierung
r_0_K = [300; 400; -400];  % [mm]
S_06_K = [-(sqrt(2)/2), 0, sqrt(2)/2; 0, 1, 0; -(sqrt(2)/2), 0, -(sqrt(2)/2)]; 
S_06_K_rot = [-(sqrt(2)/2), 0, sqrt(2)/2, 0; 0, 1, 0, 0; -(sqrt(2)/2), 0, -(sqrt(2)/2), 0; 0, 0, 0, 1]; 

% Längenangaben (Tabelle B1: Roboterparameter) [mm]
L1 = 325;
L2 = 180;
L3 = 335;
L4 = 260;


% --------------------------------------------------------------------
%%      POSITIONSPROBLEMATIK

%% Berechnung des Vektors r_0_C (Punkt C im Inertialsystem)

%   Gleichung (D.7) nach r_0_C umstellen:
%   r_0_S = r_0_C + S_06_S * r_6_CS
%   r_0_C = r_0_S - S_06_S * r_6_CS

% r_O_C berechnen für B und K
r_6_C = [0; 0; L4]; % Ortsvektor zwischen C und S im Koordinatensystem 6
r_0_C_B = r_0_B - S_06_B * r_6_C;
r_0_C_K = r_0_K - S_06_K * r_6_C;

%%  Bestimmung der Winkel alpha, beta und gamma

% Winkel alpha berechnen (D.8)
% Vorüberlegung 1: 1.Quadrant liegt innerhalb x,-z Ebene
%  -> -z entspricht y aus Gleichung (D.8)
%  -> x entspricht x aus Gleichung (D.8) 
% Vorüberlegung 2: 
% Die x- und z-Komponente von r_0_C_B sind positiv (-> 4. Quadrant)
%  -> Winkel alpha_B muss negativ und der Betrag < 90° sein
% Die x-Komponente von  r_0_C_K ist positiv, die z-Komponente negativ (1. Quadrant)
%  -> Winkel alpha_K muss positiv und der Betrag < 90° sein
alpha_B = atand( -r_0_C_B(3) / r_0_C_B(1) );
alpha_K = atand( -r_0_C_K(3) / r_0_C_K(1) );

% Winkel gamma berechnen (D.10)
gamma_B = acosd( ( r_0_C_B(3)^2 + r_0_C_B(1)^2 + ( r_0_C_B(2) - L1 )^2 - L2^2 - L3^2 ) / (2*L2*L3) );
gamma_K = acosd( ( r_0_C_K(3)^2 + r_0_C_K(1)^2 + ( r_0_C_K(2) - L1 )^2 - L2^2 - L3^2 ) / (2*L2*L3) );

% Winkel beta berechnen (D.11)
% Vorüberlegung:
% Um den Wert pi größeres beta (bzw. um 180° größerer Winkel) stellt ebenfalls
% eine Lösung dar. Winkel beta wird so gewählt, dass alle 12 Gleichung erfüllt sind.
% Vorüberlegung: um gamma wird im Uhrzeigersinn gedreht 
% -> gamma wird negativ eingesetzt
beta_B = atand( (r_0_C_B(2) - L1) / sqrt( r_0_C_B(3)^2 + r_0_C_B(1)^2 ) ) - atand( (L3 * sind(-gamma_B)) / (L2+L3*cosd(-gamma_B)) );
beta_K = 180 + atand( (r_0_C_K(2) - L1) / sqrt( r_0_C_K(3)^2 + r_0_C_K(1)^2 ) ) - atand( (L3 * sind(-gamma_K)) / (L2+L3*cosd(-gamma_K)) );


% --------------------------------------------------------------------
%%      ORIENTIERUNGSPROBLEMATIK

%%  Einsetzen der bestimmen Winkel in die symbolischen Gleichungen der homogenen Rotations-Transformationsmatrizen

% alpha 
S_01_B = [cosd(alpha_B), 0, sind(alpha_B), 0; 
                  0, 1, 0, 0; 
                  -sind(alpha_B), 0, cosd(alpha_B), 0; 
                  0, 0, 0, 1];
S_01_K = [cosd(alpha_K), 0, sind(alpha_K), 0; 
                  0, 1, 0, 0; 
                  -sind(alpha_K), 0, cosd(alpha_K), 0; 
                  0, 0, 0, 1];         
% beta
S_12_B = [cosd(beta_B),  -sind(beta_B), 0, 0; 
                  sind(beta_B), cosd(beta_B), 0, 0; 
                  0, 0, 1, 0;
                  0, 0, 0, 1];
S_12_K = [cosd(beta_K),  -sind(beta_K), 0, 0; 
                  sind(beta_K), cosd(beta_K), 0, 0; 
                  0, 0, 1, 0;
                  0, 0, 0, 1];      
% gamma
S_23_B = [cosd(gamma_B),  sind(gamma_B), 0, 0; 
                  -sind(gamma_B), cosd(gamma_B), 0, 0; 
                  0, 0, 1, 0;
                  0, 0, 0, 1];    
S_23_K = [cosd(gamma_K),  sind(gamma_K), 0, 0; 
                  -sind(gamma_K), cosd(gamma_K), 0, 0; 
                  0, 0, 1, 0;
                  0, 0, 0, 1];
              
%% Berechnung der Matrix S_03 (Gleichung D.12)

% Berechnung von S_03 allgemein (d.h. mit symbolischen Variablen)
S_03 = S_01 * S_12 * S_23;  

% Berechnung von S_03_B  und S_03_K mit eingesetzten Winkeln alpha, beta und gamma
S_03_B = S_01_B * S_12_B * S_23_B;         
S_03_K = S_01_K * S_12_K * S_23_K;        
 
%% Berechnung der Matrix S_36 (Gleichungen D.13 und D.14)

% Berechnung von S_36 allgemein (d.h. mit symbolischen Variablen)
S_36 = S_34 * S_45 * S_56;

% Berechnung von S_36_B  und S_36_K mit berechneten Matrizen S_03_B und S_03_K
% und gegebenen Matrizen S_06_B und S_06_K
% Vorüberlegung:
% S_06_S = S_03 * S_36 (D.13)                   I Transponierte Matrix S_03' von links mulitiplizieren
% S_03' * S_06_S = S_03' * S_03 * S_36
% S_36 = S_06_S * S_03' (D.14)
S_36_B = S_03_B' * S_06_B_rot; 
S_36_K = S_03_K' * S_06_K_rot;

%%  Berechnung der Winkel delta, epsilon und phi 

% Winkel eplison bestimmen
% Vorüberlegung: Gleichung, die lediglich eine Unbekannte aufweist, wird gelöst
epsilon_B = asind(S_36_B(1,3));
epsilon_K = asind(S_36_K(1,3));

% Vorüberlegung zur Bestimmung von Winkel phi und delta
% Unter Beachtung der in (D.15) aufgeführten Bedingungen werden nun systematisch
% die  möglichen Lösungen variiert bis eine Übereinstimmung (= erfolgreiche Lösung aller zwölf Gleichungen) erzielt wird

% Winkel phi bestimmen
phi_B = - acosd( S_36_B(1,1) /cosd(epsilon_B) );
phi_K = acosd( S_36_K(1,1) /cosd(epsilon_K) );

% Winkel delta bestimmen
delta_B = acosd( S_36_B(3,3) /  cosd(epsilon_B));
delta_K = - acosd( S_36_K(3,3) /  cosd(epsilon_K));


% --------------------------------------------------------------------
%%      KONTROLLE

%% 1. Vergleich: Matrizenprodukt S_03' * S_06_S (S_36_B) mit Matrix S_36_B2           

%% Matrix S_36_B2 mit ermittelten Winkel bestimmen und mit  S_36_B vergleichen 

epsilon = epsilon_B; 
phi = phi_B;
delta = delta_B;
S_36_B2 = [cosd(epsilon)*cosd(phi), -cosd(epsilon)*sind(phi), sind(epsilon), 0; 
                    cosd(delta)*sind(phi) + cosd(phi)*sind(delta)*sind(epsilon), cosd(delta)*cosd(phi) - sind(delta)*sind(epsilon)*sind(phi), -cosd(epsilon)*sind(delta), 0;
                    sind(delta)*sind(phi) - cosd(delta)*cosd(phi)*sind(epsilon), cosd(phi)*sind(delta) + cosd(delta)*sind(epsilon)*sind(phi),  cosd(delta)*cosd(epsilon), 0;
                    0, 0, 0, 1];             
% Rückmeldung zum Vergleich im Worspace anzeigen
if round(S_36_B2) == round(S_36_B)
    fprintf('Matrix S_36 richtig bestimmt :-) \n');
else 
    fprintf('Matrix S_36 ist falsch :-( \n');
end

% Matrix S_36_K2 mit ermittelten Winkel bestimmen und mit  S_36_K vergleichen 
epsilon = epsilon_K; 
phi = phi_K;
delta = delta_K;
S_36_K2 = [cosd(epsilon)*cosd(phi), -cosd(epsilon)*sind(phi), sind(epsilon), 0; 
                    cosd(delta)*sind(phi) + cosd(phi)*sind(delta)*sind(epsilon), cosd(delta)*cosd(phi) - sind(delta)*sind(epsilon)*sind(phi), -cosd(epsilon)*sind(delta), 0;
                    sind(delta)*sind(phi) - cosd(delta)*cosd(phi)*sind(epsilon), cosd(phi)*sind(delta) + cosd(delta)*sind(epsilon)*sind(phi),  cosd(delta)*cosd(epsilon), 0;
                    0, 0, 0, 1];
% Rückmeldung zum Vergleich im Worspace anzeigen
if round(S_36_K2) == round(S_36_K)
    fprintf('Richtige Lösung für den Korb :-) \n');
else 
    fprintf('Falsche Lösung für den Korb :-( \n');
end


%% 2. Vergleich: Punkt B und K (in Aufgabe gegeben versus bestimmt)

%% Einsetzen der gegebenen Längen/ermittelten Winkel in die symbolischen Transformationsmatrizen

% Einsetzen der gegebenen Längen in die homogenen Translations-Transformationsmatrizen
% Ziel: symbolische Variablen in double konvertieren
% L1 in T_12
T_12 = [1, 0, 0, 0;
               0, 1, 0, L1;
               0, 0, 1, 0;
               0, 0, 0, 1];       
% L2 in T_23
T_23 = [1, 0, 0, L2;
               0, 1, 0, 0;
               0, 0, 1, 0;
               0, 0, 0, 1];      
% L3 in T_34
T_34 = [1, 0, 0, L3;
               0, 1, 0, 0;
               0, 0, 1, 0;
               0, 0, 0, 1];      
% L4 in T_67
T_67 = [1, 0, 0, 0;
               0, 1, 0, 0;
               0, 0, 1, L4;
               0, 0, 0, 1]; 

% Einsetzen der übrigen bestimmen Winkel in die homogenen Rotations-Transformationsmatrizen
% delta 
S_34_B = [1, 0, 0, 0
              0, cosd(delta_B), -sind(delta_B), 0; 
              0, sind(delta_B), cosd(delta_B), 0;
              0, 0, 0, 1];
S_34_K = [1, 0, 0, 0
              0, cosd(delta_K), -sind(delta_K), 0; 
              0, sind(delta_K), cosd(delta_K), 0;
              0, 0, 0, 1];
% epsilon
S_45_B = [cosd(epsilon_B), 0, sind(epsilon_B), 0; 
              0, 1, 0, 0; 
              -sind(epsilon_B), 0, cosd(epsilon_B), 0; 
              0, 0, 0, 1];
S_45_K = [cosd(epsilon_K), 0, sind(epsilon_K), 0; 
              0, 1, 0, 0; 
              -sind(epsilon_K), 0, cosd(epsilon_K), 0; 
              0, 0, 0, 1];
% phi
S_56_B = [cosd(phi_B),  -sind(phi_B), 0, 0; 
              sind(phi_B), cosd(phi_B), 0, 0; 
              0, 0, 1, 0;
              0, 0, 0, 1];
S_56_K = [cosd(phi_K),  -sind(phi_K), 0, 0; 
              sind(phi_K), cosd(phi_K), 0, 0; 
              0, 0, 1, 0;
              0, 0, 0, 1];
                               
%% Matrix H berechnen, Punkt B und K und mit vorgegebenen Werten vergleichen

% H berechnen durch Multiplikation der Transformationsmatrizen
H_B = S_01_B * T_12 * S_12_B * T_23 * S_23_B * T_34 * S_34_B * S_45_B * S_56_B * T_67;
H_K = S_01_K * T_12 * S_12_K * T_23 * S_23_K * T_34 * S_34_K * S_45_K * S_56_K * T_67;

% Ortsvektor der Greiferspitze r_0 (Endpunkt von L4 im Intertialsystem) aus H entnehmen
r_0_B2 = H_B(1:3, 4);
r_0_K2 = H_K(1:3, 4);

% Drehmatrix S_06 aus H entnehmen
S_06_B2 = H_B(1:3, 1:3);
S_06_K2 = H_K(1:3, 1:3);

% Vergleich r_B_0 (gegebene Position B) mit r_0_B2 (bestimmte Position B)
% und Rückmeldung Ball Position im Workspace anzeigen
if round(r_0_B) == round(r_0_B2) 
    fprintf('Richtige Position für den Ball :-) \n');
else 
    fprintf('Falsche Position für den Ball :-( \n');
end
% Vergleich S_06_B (gegebene Orientierung B) mit S_06_B2 (bestimmte Orientierung B)
% und Rückmeldung Ball Position im Workspace anzeigen 
if round(S_06_B) == round(S_06_B2)
    fprintf('Richtige Orientierung für den Ball :-) \n');
else 
    fprintf('Falsche Orientierung für den Ball :-( \n');
end

% Vergleich r_K_0 (gegebene Position K) mit r_0_K2 (bestimmte Position K)
% und Rückmeldung Korb Position im Workspace anzeigen
if round(r_0_K) == round(r_0_K2) 
    fprintf('Richtige Position für den Korb :-) \n');
else 
    fprintf('Falsche Position für den Korb :-( \n');
end
% Vergleich S_06_K (gegebene Orientierung K) mit S_06_K2 (bestimmte Orientierung K)
% und Rückmeldung Korb Position im Workspace anzeigen 
if round(S_06_K) == round(S_06_K2)
    fprintf('Richtige Orientierung für den Korb :-) \n');
else 
    fprintf('Falsche Orientierung für den Korb :-( \n');
end
