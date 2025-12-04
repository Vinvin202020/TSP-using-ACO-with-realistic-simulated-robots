% Rayon du cercle dans lequel les lumières du pentagone seront placées
r = 1.4;

% Angles des sommets du pentagone (divisés sur 5 angles de 72 degrés)
angles = linspace(0, 2*pi, 6);  % 6 angles, car on inclut aussi le centre

% Initialiser les coordonnées des lumières
light_positions = zeros(6, 2);

% Calculer les coordonnées des 5 lumières du pentagone + 1 au centre
for i = 1:5
    light_positions(i, 1) = r * cos(angles(i)); % Coordonnée x
    light_positions(i, 2) = r * sin(angles(i)); % Coordonnée y
end

% La lumière au centre est simplement l'origine
light_positions(6, :) = [0, 0];

% Afficher les positions des lumières
disp('Coordonnées des lumières :');
disp(light_positions);

% Optionnel : Visualisation
figure;
hold on;
axis equal;
xlim([-2 2]);
ylim([-2 2]);

% Tracer les lumières
plot(light_positions(:, 1), light_positions(:, 2), 'ro', 'MarkerFaceColor', 'r');

% Tracer le pentagone
plot([light_positions(1:5, 1); light_positions(1, 1)], ...
     [light_positions(1:5, 2); light_positions(1, 2)], 'b-');

% Tracer le centre
plot(0, 0, 'go', 'MarkerFaceColor', 'g');

title('Positions des lumières (Pentagone + Centre)');
grid on;
