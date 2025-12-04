% Définir la plage des fichiers à traiter (de 1 à 10)
X_values = 1:10;

% Initialiser la matrice pour stocker les phéromones (n_patrols * n_patrols * 10)
pheromone_tables = zeros(n_patrols, n_patrols, 10);

% Boucle sur tous les fichiers
for X = X_values
    % Créer le nom du fichier à partir de l'indice X
    filename = ['pheromone_history_', num2str(X), '.csv'];
    
    % Charger les données du fichier
    data = readmatrix(filename);
    
    % Extraire la première table de phéromones (c'est-à-dire pour l'itération 1)
    start_row = 3;  % Lignées où commence l'extraction des données
    rows_per_iter = n_patrols;
    
    % Extraire la première table de phéromones
    r0 = start_row;
    r1 = r0 + rows_per_iter - 1;
    block = data(r0:r1, 1:n_patrols);
    
    % Stocker la table extraite dans la matrice 3D
    pheromone_tables(:,:,X) = block;
end

% Afficher la dimension de la matrice finale
disp(size(pheromone_tables));

average_pheromone_table = mean(pheromone_tables, 3)