# P8-2024-BT-COLLECT

Implementa, utilizando BTs, un comportamiento para un robot de manera que este busque un determinado número de entidades en el laboratorio (el número de entidades así como la entidad a buscar se trabajarán como argumentos de ROS):
1. El robot deberá de localizar la entidad y navegar hacia ella
2. Cuando la haya alcanzado, deberá de emitir un sonido y llevarla hasta el punto de recogida (argumento)
3. El proceso continúa hasta que no queden entidades por regoger

El comportamiento de llevar la entidad al punto de recogida consiste únicamente en navegar de vuelta. Por ejemplo, si la entidad a buscar es una persona, sería equivaldría a un comportamiento de guiado.
