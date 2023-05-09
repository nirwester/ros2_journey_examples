Compile with "colcon build"
source install/setup.sh

Terminal 1: ros2 run lazy_subscriber publisher
Terminal 2: ros2 run lazy_subscriber lazy_subscriber
Terminal 3: ros2 run lazy_subscriber subscriber

In terminal 2 you will observe how the node subscribes to the input data only if the subscription on Terminal 3 is active.

Example of output:

ros2 run lazy_subscriber lazy_subscriber 

# Starting the subscriber

[INFO] [1683668692.448767504] [lazy_subscriber]: Tot Matching connections: 1
[INFO] [1683668692.448843449] [lazy_subscriber]: Tot Count change: 1
[INFO] [1683668692.448854225] [lazy_subscriber]: Current connections: 1
[INFO] [1683668692.448863348] [lazy_subscriber]: Current change: 1
[INFO] [1683668692.448884488] [lazy_subscriber]: Creating subscription
[INFO] [1683668692.947240055] [lazy_subscriber]: I heard: 'Hello, world! 88' and published 'HELLO, WORLD! 88'
[INFO] [1683668693.447202165] [lazy_subscriber]: I heard: 'Hello, world! 89' and published 'HELLO, WORLD! 89'
[INFO] [1683668693.947157774] [lazy_subscriber]: I heard: 'Hello, world! 90' and published 'HELLO, WORLD! 90'
[INFO] [1683668694.447169149] [lazy_subscriber]: I heard: 'Hello, world! 91' and published 'HELLO, WORLD! 91'
[INFO] [1683668694.947173100] [lazy_subscriber]: I heard: 'Hello, world! 92' and published 'HELLO, WORLD! 92'
[INFO] [1683668695.447198212] [lazy_subscriber]: I heard: 'Hello, world! 93' and published 'HELLO, WORLD! 93'
[INFO] [1683668695.946906707] [lazy_subscriber]: I heard: 'Hello, world! 94' and published 'HELLO, WORLD! 94'
[INFO] [1683668696.447525618] [lazy_subscriber]: I heard: 'Hello, world! 95' and published 'HELLO, WORLD! 95'
[INFO] [1683668696.947187049] [lazy_subscriber]: I heard: 'Hello, world! 96' and published 'HELLO, WORLD! 96'
[INFO] [1683668697.447261313] [lazy_subscriber]: I heard: 'Hello, world! 97' and published 'HELLO, WORLD! 97'
[INFO] [1683668697.947261491] [lazy_subscriber]: I heard: 'Hello, world! 98' and published 'HELLO, WORLD! 98'
[INFO] [1683668698.447198314] [lazy_subscriber]: I heard: 'Hello, world! 99' and published 'HELLO, WORLD! 99'
[INFO] [1683668698.947234688] [lazy_subscriber]: I heard: 'Hello, world! 100' and published 'HELLO, WORLD! 100'
[INFO] [1683668699.447262908] [lazy_subscriber]: I heard: 'Hello, world! 101' and published 'HELLO, WORLD! 101'
[INFO] [1683668699.947207446] [lazy_subscriber]: I heard: 'Hello, world! 102' and published 'HELLO, WORLD! 102'
[INFO] [1683668700.447237216] [lazy_subscriber]: I heard: 'Hello, world! 103' and published 'HELLO, WORLD! 103'
[INFO] [1683668700.947211019] [lazy_subscriber]: I heard: 'Hello, world! 104' and published 'HELLO, WORLD! 104'
[INFO] [1683668701.447251620] [lazy_subscriber]: I heard: 'Hello, world! 105' and published 'HELLO, WORLD! 105'
[INFO] [1683668701.947265616] [lazy_subscriber]: I heard: 'Hello, world! 106' and published 'HELLO, WORLD! 106'
[INFO] [1683668702.447308267] [lazy_subscriber]: I heard: 'Hello, world! 107' and published 'HELLO, WORLD! 107'
[INFO] [1683668702.947244357] [lazy_subscriber]: I heard: 'Hello, world! 108' and published 'HELLO, WORLD! 108'
[INFO] [1683668703.447193851] [lazy_subscriber]: I heard: 'Hello, world! 109' and published 'HELLO, WORLD! 109'
[INFO] [1683668703.947175435] [lazy_subscriber]: I heard: 'Hello, world! 110' and published 'HELLO, WORLD! 110'
[INFO] [1683668704.447186665] [lazy_subscriber]: I heard: 'Hello, world! 111' and published 'HELLO, WORLD! 111'
[INFO] [1683668704.947130708] [lazy_subscriber]: I heard: 'Hello, world! 112' and published 'HELLO, WORLD! 112'
[INFO] [1683668705.447184271] [lazy_subscriber]: I heard: 'Hello, world! 113' and published 'HELLO, WORLD! 113'
[INFO] [1683668705.947092619] [lazy_subscriber]: I heard: 'Hello, world! 114' and published 'HELLO, WORLD! 114'
[INFO] [1683668706.447338889] [lazy_subscriber]: I heard: 'Hello, world! 115' and published 'HELLO, WORLD! 115'
[INFO] [1683668706.947243102] [lazy_subscriber]: I heard: 'Hello, world! 116' and published 'HELLO, WORLD! 116'
[INFO] [1683668707.446939748] [lazy_subscriber]: I heard: 'Hello, world! 117' and published 'HELLO, WORLD! 117'
[INFO] [1683668707.947166349] [lazy_subscriber]: I heard: 'Hello, world! 118' and published 'HELLO, WORLD! 118'
[INFO] [1683668708.447179726] [lazy_subscriber]: I heard: 'Hello, world! 119' and published 'HELLO, WORLD! 119'
[INFO] [1683668708.947181982] [lazy_subscriber]: I heard: 'Hello, world! 120' and published 'HELLO, WORLD! 120'
[INFO] [1683668709.447214533] [lazy_subscriber]: I heard: 'Hello, world! 121' and published 'HELLO, WORLD! 121'
[INFO] [1683668709.947140368] [lazy_subscriber]: I heard: 'Hello, world! 122' and published 'HELLO, WORLD! 122'
[INFO] [1683668710.447176022] [lazy_subscriber]: I heard: 'Hello, world! 123' and published 'HELLO, WORLD! 123'
[INFO] [1683668710.947173088] [lazy_subscriber]: I heard: 'Hello, world! 124' and published 'HELLO, WORLD! 124'
[INFO] [1683668711.006819453] [lazy_subscriber]: Tot Matching connections: 1
[INFO] [1683668711.006850562] [lazy_subscriber]: Tot Count change: 0
[INFO] [1683668711.006858694] [lazy_subscriber]: Current connections: 0
[INFO] [1683668711.006865480] [lazy_subscriber]: Current change: -1
[INFO] [1683668711.006871064] [lazy_subscriber]: Stopping subscription

# Stopped the subscriber and restarted it after some seconds

[INFO] [1683668738.106214218] [lazy_subscriber]: Tot Matching connections: 2
[INFO] [1683668738.106240120] [lazy_subscriber]: Tot Count change: 1
[INFO] [1683668738.106277104] [lazy_subscriber]: Current connections: 1
[INFO] [1683668738.106281756] [lazy_subscriber]: Current change: 1
[INFO] [1683668738.106286987] [lazy_subscriber]: Creating subscription
[INFO] [1683668738.447190715] [lazy_subscriber]: I heard: 'Hello, world! 179' and published 'HELLO, WORLD! 179'
[INFO] [1683668738.947145962] [lazy_subscriber]: I heard: 'Hello, world! 180' and published 'HELLO, WORLD! 180'
[INFO] [1683668739.447257475] [lazy_subscriber]: I heard: 'Hello, world! 181' and published 'HELLO, WORLD! 181'
[INFO] [1683668739.947287829] [lazy_subscriber]: I heard: 'Hello, world! 182' and published 'HELLO, WORLD! 182'
[INFO] [1683668740.447213113] [lazy_subscriber]: I heard: 'Hello, world! 183' and published 'HELLO, WORLD! 183'
[INFO] [1683668740.947212865] [lazy_subscriber]: I heard: 'Hello, world! 184' and published 'HELLO, WORLD! 184'
[INFO] [1683668741.447179218] [lazy_subscriber]: I heard: 'Hello, world! 185' and published 'HELLO, WORLD! 185'
[INFO] [1683668741.947285767] [lazy_subscriber]: I heard: 'Hello, world! 186' and published 'HELLO, WORLD! 186'
[INFO] [1683668742.447271467] [lazy_subscriber]: I heard: 'Hello, world! 187' and published 'HELLO, WORLD! 187'
[INFO] [1683668742.947269381] [lazy_subscriber]: I heard: 'Hello, world! 188' and published 'HELLO, WORLD! 188'
[INFO] [1683668743.447305746] [lazy_subscriber]: I heard: 'Hello, world! 189' and published 'HELLO, WORLD! 189'
[INFO] [1683668743.920900848] [lazy_subscriber]: Tot Matching connections: 2
[INFO] [1683668743.921002415] [lazy_subscriber]: Tot Count change: 0
[INFO] [1683668743.921034245] [lazy_subscriber]: Current connections: 0
[INFO] [1683668743.921071887] [lazy_subscriber]: Current change: -1
[INFO] [1683668743.921109646] [lazy_subscriber]: Stopping subscription

# Stopping the subscriber again