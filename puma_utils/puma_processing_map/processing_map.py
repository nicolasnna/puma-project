#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import yaml
import shutil
import os
import imageio
import time

def crop_image(img, size_length_pixel, over_size = 0 ,origin_pixels = [0,0]):
  size_side = int(size_length_pixel/2) + over_size
  min_x = origin_pixels[0] - size_side
  max_x = origin_pixels[0] + size_side
  
  min_y = origin_pixels[1] - size_side
  max_y = origin_pixels[1] + size_side
  # Validate correct measurement
  min_x = min_x if min_x>=0 else 0
  max_x = max_x if max_x<=img.shape[1] else img.shape[1]
  
  min_y = min_y if min_y>=0 else 0
  max_y = max_y if max_y<=img.shape[0] else img.shape[0]
  
  return img[min_y:max_y, min_x:max_x]

def show_image(img, name):
  cv.imshow(name, img)
  cv.waitKey(0)
  cv.destroyAllWindows()

def create_pgm_yaml_file(img, filename, origin = [0, 0]):
  imageio.imwrite(filename+".pgm", img, format='PGM')
  x_offset = ((origin[0] - X_ORIGEN) - SIZE/2 - over_px) * RESOLUTION
  y_offset = ((Y_ORIGEN - origin[1]) - SIZE/2 - over_px) * RESOLUTION
  yaml_params = dict(
    image = filename + ".pgm",
    resolution = RESOLUTION,
    origin = [
      x_offset, 
      y_offset, 
      0],
    occupied_thresh = OCCUPIED_THRESH,
    free_thresh = FREE_THRESH,
    negate = NEGATE
  )
  with open(filename+'.yaml', 'w') as yaml_file:
    yaml.dump(yaml_params, yaml_file, default_flow_style=False)

def get_inputs_from_user():
  print("Ingresa el tamaño en pixels por lado de cada corte.")
  size_length_px = int(input("> "))
  print("Ingresa el pixel de origen en eje X.")
  x_origen_px = int(input("> "))
  print("Ingresa el pixel de origen en eje Y.")
  y_origen_px = int(input("> "))
  print("Ingresa largo en px establecido para la sobreescritura.")
  over_size = int(input("> "))
  
  print("---------------------------------------------------")
  print("    Iniciando con el procesado de las imagenes     ")
  print("---------------------------------------------------")
  print()
  
  return size_length_px, x_origen_px, y_origen_px, over_size

def processing_image_and_export(x_quadrant, y_quadrant, x_origin_, y_origin_):
  filename = "map-c" + str(x_quadrant) + "_" + str(y_quadrant)
  img_crop = crop_image(img, SIZE, over_px, [x_origin_, y_origin_])
  create_pgm_yaml_file(img_crop, filename=filename, origin=[x_origin_, y_origin_])
  shutil.move(filename+'.pgm', new_path_move+"/"+filename+'.pgm')
  shutil.move(filename+'.yaml', new_path_move+"/"+filename+'.yaml')
  #print("  -Se ha creado correctamente " + filename+'.pgm' + " con los parametros en " +filename+'.yaml' +".")


# Parameter for yaml

OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.196
NEGATE = 0

if __name__ == "__main__":
  # Path
  current_directory = os.getcwd()
  folder = "after_convert"
  new_path_move = current_directory+'/'+folder
  # Load img
  print("Ingresa el nombre de la imagen a procesar. (con extension)")
  img_name = input("> ")
  img = cv.imread(img_name,cv.IMREAD_GRAYSCALE)
  try:
    print(img.shape)
  except AttributeError:
    print("No se encontro la imagen en el path especificado")
    exit()
    
  print('Resolucion a trabajar: m/px.')
  RESOLUTION = float(input("> "))  
    
  SIZE, X_ORIGEN, Y_ORIGEN, over_px = get_inputs_from_user()
  
  # Calcular cuantos cuadrantes se pueden formar
  shape = img.shape
  left_x_quadrant = int((X_ORIGEN - SIZE/2) / SIZE)
  right_x_quadrant = int((shape[1] - (X_ORIGEN + SIZE/2)) / SIZE)
  up_y_quadrant = int((shape[0] - (Y_ORIGEN + SIZE/2)) / SIZE)
  down_y_quadrant = int((Y_ORIGEN - SIZE/2) / SIZE)
  
  new_x_origin = X_ORIGEN
  new_y_origin = Y_ORIGEN
  # Origin quadrant cartesian
  print("-> Iniciando con el cuadrante de origen (0, 0):")
  time.sleep(0.2)
  processing_image_and_export(0, 0, new_x_origin, new_y_origin)
  print(" ->Cuadrante de origen (0, 0) creado y guardado en la carpeta " + folder + ".")
  print()
  
  # First quadrant cartesian
  print("-> Iniciando con el primer cuadrante (x+, y+):")
  time.sleep(1)
  img_count = 0
  for x in range(0, right_x_quadrant + 1):
    new_x_origin = X_ORIGEN + SIZE*x
    for y in range(0, up_y_quadrant + 1):
      new_y_origin = Y_ORIGEN - SIZE*y
      # Validate that is not origin
      if not (x == 0 and y == 0):
        processing_image_and_export(x, y, new_x_origin, new_y_origin)
        img_count += 1
  
  print("  -Se ha creado un total de", img_count,"imagenes.")
  print(" ->Primer cuadrante (x+, y+) creado y guardado en la carpeta " + folder + ".")
  print()
  
  # Second quadrant cartesian
  print("-> Iniciando con el segundo cuadrante (x-, y+):")
  time.sleep(1)
  img_count = 0
  for x in range(0, left_x_quadrant + 1):
    new_x_origin = X_ORIGEN + SIZE*-x
    for y in range(0, up_y_quadrant + 1):
      new_y_origin = Y_ORIGEN - SIZE*y
      # Validate that is not origin
      if not (x == 0 and y == 0):
        processing_image_and_export(-x, y, new_x_origin, new_y_origin)
        img_count += 1
  
  print("  -Se ha creado un total de", img_count,"imagenes.")
  print(" ->Segundo cuadrante (x-, y+) creado y guardado en la carpeta " + folder + ".")
  print()
  
  # Third quadrant cartesian
  print("-> Iniciando con el tercer cuadrante (x-, y-):")
  time.sleep(1)
  img_count = 0
  for x in range(0, left_x_quadrant + 1):
    new_x_origin = X_ORIGEN + SIZE*-x
    for y in range(0, down_y_quadrant + 1):
      new_y_origin = Y_ORIGEN - SIZE*-y
      # Validate that is not origin
      if not (x == 0 and y == 0):
        processing_image_and_export(-x, -y, new_x_origin, new_y_origin)
        img_count += 1
  
  print("  -Se ha creado un total de", img_count,"imagenes.")
  print(" ->Tercer cuadrante (x-, y-) creado y guardado en la carpeta " + folder + ".")
  print()
  
  # Fourth quadrant cartesian
  print("-> Iniciando con el cuarto cuadrante (x+, y-):")
  time.sleep(1)
  img_count = 0
  for x in range(0, right_x_quadrant + 1):
    new_x_origin = X_ORIGEN + SIZE*x
    for y in range(0, down_y_quadrant + 1):
      new_y_origin = Y_ORIGEN - SIZE*-y
      # Validate that is not origin
      if not (x == 0 and y == 0):
        processing_image_and_export(x, -y, new_x_origin, new_y_origin)
        img_count += 1
  
  print("  -Se ha creado un total de", img_count,"imagenes.")
  print(" ->Cuarto cuadrante (x+, y-) creado y guardado en la carpeta " + folder + ".")
  print()
  
  print("------------------------------------------------")
  print("      Procesamiendo de imagenes terminado       ")
  print("------------------------------------------------")
  time.sleep(3)

