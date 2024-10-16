clc
clear all
close all

foregroundDetector = vision.ForegroundDetector('NumGaussians', 5, ...
                                               'NumTrainingFrames', 40, ...
                                               'LearningRate', 0.005, ...
                                               'MinimumBackgroundRatio', 0.7);
prev_x = 0;
prev_y = 0;
k_v = 13;

velocidades=[];

%% A. Video Input
videoSource  = VideoReader("prueba03_1.mp4");
centroides_x = [];
fps = videoSource.FrameRate;
while hasFrame(videoSource)
    %% B. Preprocessing
    img  = readFrame(videoSource); 
    img_pre = im2gray(img); % Convertir a escala de grises
    img_pre = imadjust(img_pre, [0.2 0.7], [0 1], 1.8); %Si
    %img_pre = medfilt2(img_pre, [3 3]);
    
    %% C. Background Subs (restar el fondo del frame actual)
    bg_sub = step(foregroundDetector,img_pre);

    %% D. Smoothing
    img_suav = medfilt2(bg_sub,[4 4]);

    %% E. Shadow removal
    %img_suav(img_suav == 127) = 0;  % Marcar las sombras como fondo
    %img_suav(img_suav == 255) = 1;  % Los objetos en movimiento se quedan como primer plano

    %% F. Operaciones morfológicas para eliminar ruido
    img_morfo = imopen(img_suav, strel('square', 15));
    img_morfo = imclose(img_morfo, strel('square', 30));
    imshow(img_morfo);

    %
    %% G. Object detection
    % Etiquetar las regiones conectadas
    [labels, num] = bwlabel(img_morfo);

    % Obtener propiedades de cada objeto detectado
    stats = regionprops(labels, 'BoundingBox', 'Centroid','Area');

    imshow(img); % Mostrar el frame original
    hold on;
    
    %% H. Labeling and tracking
    % Dibujar los rectángulos y centroides en los objetos detectados
    for i = 1:num
        if(stats(i).Area > 8000)
            %fprintf('Area: %.2f\n', stats(i).Area)

            % Extraer el rectángulo y el centroide
            bbox = stats(i).BoundingBox; % Coordenadas del rectángulo [x, y, width, height]
            centroide = stats(i).Centroid; % Coordenadas [x, y]
            
            % Dibujar el rectángulo delimitador
            rectangle('Position', bbox, 'EdgeColor', 'y', 'LineWidth', 2);
            
            % Dibujar el centroide
            plot(centroide(1), centroide(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
            
            %fprintf('Objeto %d: Centroide en (x, y) = (%.2f, %.2f)\n', i, centroide(1), centroide(2));
            %centroides_x = [centroides_x; centroide(1)];
    
            d = sqrt((centroide(1) - prev_x)^2 + (centroide(2) - prev_y)^2);
            d = d*0.001;
            v = k_v*(d/(1/fps));
            v = v*3.6;
            prev_x = centroide(1);
            prev_y = centroide(2);
            %fprintf('Velocidad: %.2f\n', v)
            velocidades = [velocidades,v];
        end
        
    end
    
    hold off;
    drawnow; % Actualizar el gráfico en cada iteración
end

%figure
%plot(centroides_x)
figure
velocidades(1) = [];
stem(velocidades)
fprintf('Velocidad del objeto: %.2f\n', mean(velocidades))