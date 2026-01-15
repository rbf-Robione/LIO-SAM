# NavSatFix to Odometry Converter

Bu node, `sensor_msgs/NavSatFix` (GPS) mesajlarını `nav_msgs/Odometry` mesajlarına çevirir.

## Özellikler

- GPS koordinatlarını (lat/lon/alt) yerel Kartezyen koordinatlara (x/y/z) dönüştürür
- İlk GPS fix'ini otomatik olarak origin (başlangıç noktası) olarak kullanır
- WGS84 elipsoid modelini kullanarak hassas dönüşüm yapar
- GPS covariance bilgisini odometry covariance'ına aktarır
- Pozisyon değişiminden hız tahmini yapar
- GPS fix kalitesini kontrol eder

## Kullanım

### Node'u başlatma

```bash
# ROS 2 workspace'i source et
source install/setup.bash

# Node'u başlat
ros2 run lio_sam lio_sam_navsat_to_odom
```

### Launch dosyası ile başlatma

```bash
ros2 launch lio_sam navsat_to_odom.launch.py
```

## Parametreler

| Parametre | Tip | Varsayılan | Açıklama |
|-----------|-----|------------|----------|
| `input_topic` | string | `/gps/fix` | GPS mesajlarının alınacağı topic |
| `output_topic` | string | `/gps/odometry` | Odometry mesajlarının yayınlanacağı topic |
| `frame_id` | string | `odom` | Odometry frame ID |
| `child_frame_id` | string | `base_link` | Child frame ID |
| `use_first_fix_as_origin` | bool | `true` | İlk GPS fix'i origin olarak kullan |

## Topic'ler

### Subscribe
- `input_topic` (`sensor_msgs/NavSatFix`): GPS fix mesajları

### Publish
- `output_topic` (`nav_msgs/Odometry`): Dönüştürülmüş odometry mesajları

## Koordinat Sistemi

Node, GPS koordinatlarını ENU (East-North-Up) koordinat sistemine çevirir:
- **X**: Doğu yönü (East)
- **Y**: Kuzey yönü (North)  
- **Z**: Yukarı yönü (Up)

## Örnekler

### Özel parametrelerle çalıştırma

```bash
ros2 run lio_sam lio_sam_navsat_to_odom --ros-args \
  -p input_topic:=/my_gps/fix \
  -p output_topic:=/my_odom \
  -p frame_id:=map \
  -p child_frame_id:=gps_link
```

### Topic'leri görüntüleme

```bash
# GPS mesajlarını göster
ros2 topic echo /gps/fix

# Odometry mesajlarını göster
ros2 topic echo /gps/odometry
```

## Notlar

- Node, ilk geçerli GPS fix'ini origin olarak kullanır
- GPS fix kalitesi `STATUS_FIX` veya daha iyi olmalıdır
- Hız hesaplaması basit türev yaklaşımı kullanır (1 saniyeden kısa zaman aralıkları için)
- Oryantasyon bilgisi GPS'te olmadığı için identity quaternion kullanılır
