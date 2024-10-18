% Implementación del algoritmo de estimación de la velocidad de un vehículo

% Paper de referencia:
% "Speed Estimation On Moving Vehicle Based On Digital Image Processing"
% (https://www.researchgate.net/publication/317312246)

% Autores: Oscar Ortiz Torres, Yonathan Romero Amador y Emmanuel Lechuga
% Arreola

clc
clear all
close all

%% 0. Definición de constantes, objetos y variables
Tv = 90; % Ángulo
H = 1.7; % Altura de la cámara en m
v = 0.012; %% Dimensión vertical del formato de imagen
f = 0.003; % Distancia focal de la cámara

foregroundDetector = vision.ForegroundDetector('NumGaussians', 5, ...
                                               'NumTrainingFrames', 40, ...
                                               'LearningRate', 0.005, ...
                                               'MinimumBackgroundRatio', 0.7);
prev_centroide = [0 0];
vel_vector=[];

%% A. Video Input
videoFuente  = VideoReader("oscar_20_1.mp4");

fps = videoFuente.FrameRate;

Tc = 2*atan(v/(2*f)); % Campo de visión de la cámara
T = Tv + Tc/2; % Ángulo total de visión (suma del ángulo de la cámara y el ángulo de proyección)
D = H*tan(T); % Distancia de la cámara al objeto
P = 2*tan(Tc/2)*sqrt(H^2+D^2); % Proyección del campo de visión (en metros)
I_height = videoFuente.Height; % Altura del video en pixeles
k = P / I_height; % Factor de conversión de pixeles a metros

% Creación del video de salida
videoSalida = VideoWriter('prueba_rojo20.mp4', 'MPEG-4');
videoSalida.FrameRate = fps;
open(videoSalida); % Abrir el archivo de video para escribir

while hasFrame(videoFuente)
    %% B. Preprocessing
    % Lectura de frame, se convierte de RGB a escala de grises y se ajusta
    % su contraste
    img  = readFrame(videoFuente);
    img_pre = im2gray(img);
    img_pre = imadjust(img_pre, [0.2 0.7], [0 1], 1.8);
    
    %% C. Background Subs (restar el fondo del frame actual) & E. Shadow Removal
    % Resta el fondo del frame actual usando el detector, dando una imagen
    % binaria como salida
    bg_sub = step(foregroundDetector,img_pre);

    %% D. Smoothing
    % Aplicación de filtro para reducir el ruido a la imagen
    img_suav = medfilt2(bg_sub,[4 4]);

    %% F. Operaciones morfológicas para eliminar ruido
    % Realiza una apertura (para eliminar objetos pequeños)
    % y un cierre (para rellenar huecos en los objetos)
    img_morfo = imopen(img_suav, strel('square', 15));
    img_morfo = imclose(img_morfo, strel('square', 30));

    %% G. Object detection
    % Guardado de objetos y extracción de propiedades
    [labels, num] = bwlabel(img_morfo);
    stats = regionprops(labels, 'BoundingBox', 'Centroid','Area');
    
    %% H. Labeling and tracking
    % Identificación y seguimiento de los objetos detectados
    imshow(img); % Muestra el frame original
    hold on;

    for i = 1:num
        if(stats(i).Area > 8000)
            % Extracción de coordenadas de rectángulo y centroide
            bbox = stats(i).BoundingBox;
            centroide = stats(i).Centroid;
            
            % Dibujo de rectángulos y centroides en los objetos detectados
            rectangle('Position', bbox, 'EdgeColor', 'y', 'LineWidth', 2);
            plot(centroide(1), centroide(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
            
            %% I. Speed Estimation
            % Cálculo de la distancia Euclidiana respecto al frame anterior
            d_e = sqrt(sum((prev_centroide-centroide).^2));
            
            % Cálculo de la velocidad en km/h
            v = 3.6*(k*(d_e/(1/fps)));
            vel_vector = [vel_vector,v];

            % Guardado de coordenadas para la siguiente iteración
            prev_centroide(1) = centroide(1);
            prev_centroide(2) = centroide(2);

            % Impresión de la velocidad encima del objeto
            texto_vel = sprintf('Vel: %.2f km/h', v);
            text(bbox(1), bbox(2)-10, texto_vel, 'Color', 'blue', 'FontSize', 12, 'FontWeight', 'bold', ...
                 'BackgroundColor', 'yellow');
        end
    end
    
    hold off;
    drawnow; % Actualización el gráfico en cada iteración
    
    % Escritura del frame procesado en el video de salida
    frame = getframe(gca);
    writeVideo(videoSalida, frame);
end

vel_vector(1) = []; % Eliminación de la primera muestra

% Impresión de la velocidad promedio del objeto
texto_velf = sprintf('Velocidad del objeto: %.2f km/h', mean(vel_vector));
text(10, 30, texto_velf, 'Color', 'blue', 'FontSize', 12, 'FontWeight', 'bold', ...
                 'BackgroundColor', 'yellow');

% Escritura del frame final
frame = getframe(gca);
writeVideo(videoSalida, frame);
close(videoSalida);