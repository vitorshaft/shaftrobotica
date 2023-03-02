'''
Script de calibração de câmera desenvolvido a partir de tutorial do OpenCV
https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

Vitor Domingues
02-fev-2023
https://www.instagram.com/shaftrobotica/
'''

import cv2
import numpy as np
import pathlib
import glob

#amostras coletadas em um tabuleiro 8x8 com casas de 2,4cm
pasta = "/coppeliaCamCalib/"
extensao = ".jpg"
tamQuad = 2.4   #tamanho de cada casa do tabuleiro
#numero de vertices do tabuleiro:
larg = 7
alt = 7

def calibrate_chessboard(dir_path, image_format, square_size, width, height):
    '''Calibrate a camera using chessboard images.'''
    # criterio de terminação
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # prepara pontos no objeto com (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * square_size

    # Arrays para armazenar pontos no objeto e na imagem a partir de todas as fotos.
    objpoints = []  # pontos 3D no mundo real.
    imgpoints = []  # pontos 2D no plano da imagem.

    images = glob.glob('<endereco da pasta de fotos tiradas>/*.jpg')
    #Itera por todas as imagens
    for fname in images:
        
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Procura os cantos do tabuleiro de xadrez
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

        # Se econtrar, adiciona pontos do objeto e pontos da imagem (após refiná-los)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Desenha e exibe os cantos
            cv2.drawChessboardCorners(img, (8,8), corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)

    # Calibra a camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    return [ret, mtx, dist, rvecs, tvecs]

val = calibrate_chessboard(pasta,extensao,tamQuad,larg,alt)
cv2.destroyAllWindows()

#exibe a matriz da camera:
print(val[1])

#substituir com elementos da matriz da camera:
def tImagem(x_im, y_im, z_real):
    c_u = 3.03978608e+02    #1ª linha, 3ª coluna
    c_v = 3.31955950e+02    #2ª linha, 3ª coluna
    imAlpha = 1.0920372067  #c_v/c_u
    f = 1.28601394e+03      #media aritmetica entre fx (1ª linha, 1ª coluna) e fy (2ª linha, 2ª coluna)
    x_real = (z_real*(x_im-c_u))/(f*imAlpha)
    y_real = (z_real*(y_im-c_v))/(f)
    return([x_real,y_real,z_real])

print(tImagem(640,260,0.985)) #z_real e a distancial real em metros