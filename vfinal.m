clc
clear all
close all

%% 0. Definición de constantes, objetos y variables
Tv = 90; % Ángulo
H = 1.7; % Altura de la cámara en m
v = 0.012; %% Dimensión vertical del formato de imagen
f = 0.003; % Distancia focal de la cámara (8-25)

Tc = 2*atan(v/(2*f));
T = Tv + Tc/2;
D = H*tan(T);
P = 2*tan(Tc/2)*sqrt(H^2+D^2);
I_height = 1080; % Alto en pixeles de la imagen
k = P / I_height;

foregroundDetector = vision.ForegroundDetector('NumGaussians', 5, ...
                                               'NumTrainingFrames', 40, ...
                                               'LearningRate', 0.005, ...
                                               'MinimumBackgroundRatio', 0.7);
pre = [0 0];
vel_vector=[];

%% A. Video Input
videoSource  = VideoReader("oscar_20_1.mp4");
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
            
            %% I. Speed Estimation
            d = sqrt(sum((pre-centroide).^2));
            v = k*(d/(1/fps));
            v = 3.6*v;

            pre(1) = centroide(1);
            pre(2) = centroide(2);
            vel_vector = [vel_vector,v];

            % Obtener la posición del texto (encima del rectángulo)
            pos_x = bbox(1); % Coordenada X del rectángulo
            pos_y = bbox(2) - 10; % Coordenada Y ligeramente encima del rectángulo

            % Mostrar la velocidad encima del objeto con fondo
            texto_vel = sprintf('Vel: %.2f km/h', v);
            text(pos_x, pos_y, texto_vel, 'Color', 'yellow', 'FontSize', 12, 'FontWeight', 'bold', ...
                 'BackgroundColor', 'black', 'EdgeColor', 'yellow', 'Margin', 3);
        end
    end
    
    hold off;
    drawnow; % Actualizar el gráfico en cada iteración
end

%figure
%plot(centroides_x)
%figure
vel_vector(1) = [];
%stem(vel_vector)
fprintf('Velocidad del objeto: %.2f\n', mean(vel_vector))