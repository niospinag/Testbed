# Testbed

antes de Inicilizar el laboratorio es recomendable crear un virtual environment para evitar errores entre librerias, ademas, el laboratorio esta implementado en python 3.10 debido a su manejo optimo de errores

## creacion de virtual environment con conda
 para crear el virtual env y cargar las dependencias es necesario correr el siguiente codigo
  

# datos a modificar en cada laboratorio
es necesario definir los puertos de comunicacion con las antenas y el numero de la camara que se va a usar

1. camara
en el archivo 'Testbed_real' es importante cambiar en la linea 81 el numero de la camara 
``` python
        self.cap = cv2.VideoCapture(0)
```

donde el numero representa que camara se piensa usar
ex:
- 0: frontal 
- 1: trasera 
- 2: externa

2. antenas de comunicacion
para comunicarse con las antenas wifi es necesario especificar cuales son los puertos seriales de comunicacion.
En 'Testbed_real' en las lineas 105-108 es necesario especificar el puerto
si usas windows
``` python
    self.esp8266 = serial.Serial("COM4", 115200)
    if number_of_robots > 6:
        self.esp8266_2 = serial.Serial("COM5", 115200)
```


En UBUNTU

``` python
    self.esp8266 = serial.Serial("/dev/ttyUSB0", 115200)
    if number_of_robots > 6:
        self.esp8266_2 = serial.Serial("/dev/ttyUSB1", 115200)
```

NOTA: si se van a realizar mas d e8 robots es necesario conetar dos antenas

si usas ubuntu y no hay acceso al puerto com debes correr el siguiente comando 
``` shell
sudo chmod 666 /dev/ttyUSB0
```

# ajustar la simulacion
es necesario que cada prueba el usuario modifique 2 scripts importantes:
1. el script utilities.misc con el fin de cargar los datos como los tenga guardados (.mat, .csv, .db, etc) actualmente hay una funcion ejemplo llamada load_data_matlab() pero puede ser modificada con el fin de cargar los datos a gusto
2. el script utilities.controllers donde se describe el algoritmo de control del cual se vaya a implementar.

ademas se debe ajustar los nombres tanto de los datos que se van a cargar
'''python 
data_name = 'data_7v_7N' #name of the simulation
'''

como tambien del nombre del video que se guardara en la carpeta video
```python 
r.record_video('video_data_' + data_name)
```

