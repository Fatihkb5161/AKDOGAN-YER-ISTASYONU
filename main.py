# TEKNOFEST INTERNATIONAL UNMANNED VEHICLE COMPETITION
# TRABZON KTU UZAY AKDOGAN TEAM

#                                                                          * IMPORT FILES *
# ----- GUI -----
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QVBoxLayout
from gui import Ui_MainWindow
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, QSettings

# ----- MAP -----
import folium

# ----- AUTONOMOUS FLIGHT -----
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
from pymavlink import mavutil


# ----- SYSTEMS LIBRARIES -----
import serial.tools.list_ports
import requests
ports = serial.tools.list_ports.comports()
import time
from datetime import datetime as dt
import math
import pytz
from pathlib import Path

# ----- AVIONIC INSTRUMENTS -----
from qfi import qfi_ADI, qfi_HSI, qfi_ALT, qfi_SI

# ----- OTHERS LIBRARIES -----
from telemetri import Telemetri
from ayarlar import Ui_Settings


#                                                                          * GLOBAL VARIABLES *

ilk_mesaj = True
harita_yenileme = 2000
sunucu_yenileme = 1
baglanilan_iha = None
iha_turu = None

destek_iha_port = "udpin:0.0.0.0:14550"  # "COM6"
destek_iha_baud = 115200
destek_iha_connected = False

imha_iha_port = "udpin:0.0.0.0:14550"
imha_iha_baud = 115200
imha_iha_connected = False

simulasyon_port = "udpin:0.0.0.0:14550"
simulasyon_baud = 115200
simulasyon_connected = False

destek_iha = None
imha_iha = None
simulasyon_iha = None

veriler = None
connected = False
last_coor = [None, None]
hedef_coor = [0.0, 0.0]  # Akıllı saatten gelen konum bilgisi.
ev_coor = [None, None]  # Başlangıç konumu
url = "https://api.thingspeak.com/channels/2461285/feeds.json?api_key=E7EEPQ5DCR5IVD46&results=1"
map_url = "map.html"

saatler = []
timer = QTimer()

durdur_btn_clicked = False
iptal_btn_clicked = False

iha_gorevde = False  # İHA'lardan birinin görevde olup olmadığının kontrolü.

#                                                                          * FILE PATHS *
resources_path = Path("Resources").resolve()
home_icon_path = f'{resources_path / "homeicon.png"}'
destek_iha_path = f'{resources_path / "yardimlogo.png"}'
imha_iha_path = f'{resources_path / "imhaLogo.png"}'
map_path = f'{Path(__file__).parent / "map.html"}'
hedef_konum_icon = f'{resources_path / "hedefkonum.png"}'


#                                                                          * MILITARY SMARTWATCH *
def haversine(lat1, lon1, lat2, lon2):
    """İki konum arasındaki mesafeyi ölçen fonksiyon"""

    # Dünyanın yarıçapı (kilometre cinsinden)
    r = 6371.0

    # Dereceyi radyana çevirme
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    # Farkları hesapla
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # Haversine formülü
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = r * c

    return round(distance * 1000, 4)  # metre cinsinden


def start_timer(iha_to_go, gidis_yonu):
    """Timer eğer zamanlayici fonksiyonuna bağlıysa bağlantıyı keser
       bağlı değilse yeniden bağlanır. Ve timer başlatılır."""

    global timer
    try:
        timer.timeout.disconnect()
    except TypeError:
        pass  # Eğer timeout zaten bağlı değilse TypeError alırız, bu durumda pass ile geçiyoruz.

    timer.timeout.connect(lambda: zamanlayici(iha_to_go, gidis_yonu))
    timer.start()
    ui.mesajlarList.addItem(f"{dt.now().strftime('%H.%M.%S')} =>  Uçuş için geri sayım başladı.")


def zamanlayici(iha_to_go, gidis_yonu):
    """ Akıllı saatten gelen iha bilgisine göre kalkacak olan İHA'nın uçuş kodu başlatılır."""
    global zamanlayici_counter, iptal_btn_clicked, veri_yaz, connected, destek_iha, imha_iha, destek_iha_connected, imha_iha_connected, iha_turu

    if not iptal_btn_clicked:

        if zamanlayici_counter > 0:
            zamanlayici_counter -= 1
            ui.ucus_suresi.setText(str(zamanlayici_counter))

        elif zamanlayici_counter == 0:
            ui.ucus_suresi.setText("Uçuş Başlıyor!")

            if iha_to_go == 1:
                print("a")
                if not veri_yaz:
                    iha_turu = 1
                    telemetry(0)
                    veri_yaz = True
                    ui.mesajlarList.addItem(f"{dt.now().strftime('%H.%M.%S')} =>  Destek verileri yazdırılmaya başlandı.")

                print("Destek verileri yazdırılmaya başlandı.")
                if gidis_yonu == 0:
                    destek(hedef_coor[0], hedef_coor[1])
                    print("Destek fonksiyonu tamamlandı.")
                elif gidis_yonu == 1:
                    print("donus basladı")
                    destek_donus()

            elif iha_to_go == 0:
                if not veri_yaz:
                    iha_turu = 0
                    telemetry(1)
                    veri_yaz = True
                    ui.mesajlarList.addItem(f"{dt.now().strftime('%H.%M.%S')} =>  İmha verileri yazdırılmaya başlandı.")

                print("İmha verileri yazdırılmaya başlandı.")
                imha(hedef_coor[0], hedef_coor[1])


            timer.stop()
            zamanlayici_counter = int(second_ui.ucusGeriSayimSuresi.value())
    else:
        print("Uçuş iptal edildi.")
        print("Uçuş iptal edildi.")
        ui.mesajlarList.addItem(f"{dt.now().strftime('%H.%M.%S')} =>  Uçuş iptal edildi!")
        iptal_btn_clicked = False
        timer.stop()
        ui.ucus_suresi.setText(str(second_ui.ucusGeriSayimSuresi.value()))
        zamanlayici_counter = int(second_ui.ucusGeriSayimSuresi.value())

    print(f"zamanlayıcı: {zamanlayici_counter}")


def yerel_zaman_donustur(zaman):
    """ Sunucudan gelen oluşturulma zamanı yerel zamana çevirilir. """
    trt = pytz.timezone('Europe/Istanbul')

    # ThingSpeak'den gelen örnek UTC zaman damgası
    utc_time_str = zaman

    # UTC zaman damgasını datetime nesnesine dönüştürün
    utc_time = dt.strptime(utc_time_str, '%Y-%m-%dT%H:%M:%SZ')

    # datetime nesnesini UTC olarak yerelleştirin
    utc_time = pytz.utc.localize(utc_time)

    # UTC zamanını TRT'ye dönüştürün
    trt_time = utc_time.astimezone(trt)

    return trt_time.strftime('%Y-%m-%d %H:%M:%S').split()[0]


def akilli_saat_bilgilerini_yazdir(mesaj):
    """Hangi ihanın geldiği bilgisi kontrol edilir ve kullanıcı arayüzü kısmına gelen bilgiler eklenir."""
    global hedef_coor, sunucu_yenileme, ilk_mesaj

    try:
        feed = mesaj['feeds'][0]
        zaman = int(feed['field4'])
        back = int(feed['field5'])

        now = dt.now()
        now_date = now.date()
        suan_saat = now.hour  # son bir saat kontrolü için

        yeni_created_at = yerel_zaman_donustur(feed['created_at'])

        if yeni_created_at == str(now_date) and back == 0:
            if ilk_mesaj:
                ilk_mesaj = False

            else:
                if zaman not in saatler:
                    sn = int(zaman % 60)
                    zaman /= 60
                    dk = int(zaman % 60)
                    saat = int(zaman / 60)
                    timer.setInterval(1000)
                    if feed['field3'] == '1':
                        my_message = f"İHA: Destek, LAT: {feed['field1']}, LONG: {feed['field2']}, SAAT: {saat}.{dk}.{sn}"
                        ui.listWidget.addItem(my_message)
                        ui.mesajlarList.addItem(f"{dt.now().strftime('%H.%M.%S')} =>  Destek İHA'sı için istek geldi.")
                        if not iha_gorevde:
                            hedef_coor[0] = float(feed['field1'])
                            hedef_coor[1] = float(feed['field2'])
                            start_timer(1, 0)

                    elif feed['field3'] == '0':
                        my_message = f"İHA: İmha, LAT: {feed['field1']}, LONG: {feed['field2']}, SAAT: {saat}.{dk}.{sn}"
                        ui.listWidget.addItem(my_message)
                        ui.mesajlarList.addItem(f"{dt.now().strftime('%H.%M.%S')} =>  İmha İHA'sı için istek geldi.")
                        if not iha_gorevde:
                            hedef_coor[0] = float(feed['field1'])
                            hedef_coor[1] = float(feed['field2'])
                            start_timer(0, 0)
                    else:
                        print(f"akilli_saat_bilgilerini_yazdir:  akıllı saatten iha bilgisi alınamadı.")

            saatler.append(int(feed["field4"]))

        elif yeni_created_at == str(now_date) and back == 1:
            if ilk_mesaj:
                ilk_mesaj = False
            else:
                if zaman not in saatler:
                    sn = int(zaman % 60)
                    zaman /= 60
                    dk = int(zaman % 60)
                    saat = int(zaman / 60)
                    timer.setInterval(1000)
                    if feed['field3'] == '1':
                        my_message = f"İHA: Destek, LAT: {feed['field1']}, LONG: {feed['field2']}, SAAT: {saat}.{dk}.{sn}"
                        ui.listWidget.addItem(my_message)
                        ui.mesajlarList.addItem(f"{dt.now().strftime('%H.%M.%S')} =>  Destek İHA'sı için geri dönüş onayı geldi.")
                        hedef_coor[0] = float(feed['field1'])
                        hedef_coor[1] = float(feed['field2'])
                        start_timer(1, 1)

                    elif feed['field3'] == '0':
                        my_message = f"İHA: İmha, LAT: {feed['field1']}, LONG: {feed['field2']}, SAAT: {saat}.{dk}.{sn}"
                        ui.listWidget.addItem(my_message)
                        ui.mesajlarList.addItem(f"{dt.now().strftime('%H.%M.%S')} =>  İmha İHA'sı için geri dönüş onayı geldi.")
                        hedef_coor[0] = float(feed['field1'])
                        hedef_coor[1] = float(feed['field2'])
                        start_timer(0, 1)
            saatler.append(int(feed["field4"]))

    except Exception as err:
        print(f"akilli_saat_bilgilerini_yazdir: {err}")


class MessagesThread(QThread):
    progress = pyqtSignal(dict)
    finished = pyqtSignal()

    def run(self):
        global url
        while True:
            try:
                response = dict(requests.get(url).json())
                self.progress.emit(response)

            except Exception as err:
                print(f"MessagesThread: {err}")
                print(f"MessagesThread: {err}")


# Haritanın kodları
map_timer = QTimer()
map_timer.setInterval(2000)
map_timer.timeout.connect(lambda: update(veriler))

class Map(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.latitude = 40.9929858
        self.longtitude = 39.7738606
        self.setGeometry(QtCore.QRect(0, 0, 1500, 1500))
        self.setObjectName("mapFrame")
        self.m = folium.Map(
            zoom_start=20,
            location=[self.latitude, self.longtitude],
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr='Esri',
            name='Esri Satellite',
            overlay=False,
            control=True,
        )
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.home_icon = folium.features.CustomIcon(home_icon_path, icon_size=(25, 25))

        self.destek_iha_icon = folium.features.CustomIcon(destek_iha_path, icon_size=(50, 50))
        self.imha_iha_icon = folium.features.CustomIcon(imha_iha_path, icon_size=(50, 50))
        self.destek_marker = folium.Marker(location=[self.latitude, self.longtitude], icon=self.destek_iha_icon).add_to(self.m)
        self.home_marker = folium.Marker(location=[-35.36329373, 149.16523741], icon=self.home_icon).add_to(self.m)
        # self.imha_marker = folium.Marker(location=[self.latitude, self.longtitude], icon=self.imha_iha_icon).add_to(self.m)

        self.m.save(map_path)
        self.webView = QWebEngineView()
        self.webView.load(QtCore.QUrl.fromLocalFile(map_path))
        self.layout.addWidget(self.webView)
        self.setLayout(self.layout)


first = True

def update(veri):
    global last_coor, first, map_url, hedef_konum_icon, iha_turu
    if veri is not None:
        if iha_turu == 1:
            if veri.lat != last_coor[0] or veri.long != last_coor[1]:
                try:
                    ui.my_map.m.zoom_start = 45
                    ui.my_map.m.location = [veri.lat, veri.long]  # burayı yorum satırına alıp dene
                    ui.my_map.destek_marker.location = [veri.lat, veri.long]
                    try:
                        if not hedef_coor[0] is None and not hedef_coor[1] is None and first:
                            hedef_konum_icon = folium.features.CustomIcon(hedef_konum_icon, icon_size=(50, 50))
                            print(f"hedef konum: ({hedef_coor[0], hedef_coor[1]})")
                            hedef_konum_marker = folium.Marker(location=[hedef_coor[0], hedef_coor[1]], icon=hedef_konum_icon).add_to(ui.my_map.m)
                            folium.PolyLine([(veri.lat, veri.long), (hedef_coor[0], hedef_coor[1])],
                                            color="blue").add_to(ui.my_map.m)
                            first = False
                    except Exception as e:
                        print(e)
                    ui.my_map.m.save(map_path, close_file=False)
                    ui.my_map.webView.load(QtCore.QUrl.fromLocalFile(map_path))
                    ui.my_map.layout.addWidget(ui.my_map.webView)
                    ui.my_map.setLayout(ui.my_map.layout)
                    last_coor = [veri.lat, veri.long]

                except Exception as err:
                    print(f"update map: {err}")
        elif iha_turu == 0:
            if veri.lat != last_coor[0] or veri.long != last_coor[1]:
                try:
                    ui.my_map.m.zoom_start = 45
                    ui.my_map.m.location = [veri.lat, veri.long]  # burayı yorum satırına alıp dene
                    ui.my_map.destek_marker.location = [veri.lat, veri.long]
                    try:
                        if not hedef_coor[0] is None and not hedef_coor[1] is None and first:
                            hedef_konum_icon = folium.features.CustomIcon(hedef_konum_icon, icon_size=(50, 50))
                            print(f"hedef konum: ({hedef_coor[0], hedef_coor[1]})")
                            hedef_konum_marker = folium.Marker(location=[hedef_coor[0], hedef_coor[1]], icon=hedef_konum_icon).add_to(ui.my_map.m)
                            folium.PolyLine([(veri.lat, veri.long), (hedef_coor[0], hedef_coor[1])],
                                            color="blue").add_to(ui.my_map.m)
                            first = False
                    except Exception as e:
                        print(e)
                    ui.my_map.m.save(map_path, close_file=False)
                    ui.my_map.webView.load(QtCore.QUrl.fromLocalFile(map_path))
                    ui.my_map.layout.addWidget(ui.my_map.webView)
                    ui.my_map.setLayout(ui.my_map.layout)
                    last_coor = [veri.lat, veri.long]
                except Exception as err:
                    print(f"update map: {err}")
    else:
        print("Veriler None")

class MapThread(QThread):
    progress = pyqtSignal()
    finished = pyqtSignal()

    def run(self):
        try:
            self.progress.emit()
        except Exception as err:
            print(f"MapThreadError: {err}")


rtl_not_clicked = True


# Telemetri verilerinin yazdırılması
veri_yaz = False


def make_not_running():
    global veri_yaz, destek_iha, imha_iha
    veri_yaz = False
    ui.yazdirBtn.setEnabled(True)
    ui.mesajlarList.addItem(f"{dt.now().strftime('%H.%M.%S')} =>  Verileri yazdırma işlemi durduruldu.")


#                                                                          * PRINTING TELEMETRY DATA *
def connect_iha(port, baud, uav1):
    """It returns the iha"""
    try:
        uav = mavutil.mavlink_connection(port, baud, autoreconnect=True)
        uav.wait_heartbeat()
        print("İHA'ya bağlanıldı.")
        return uav
    except Exception as e:
        print("İHA'ya bağlanılamadı.")
        return uav1


class TelemetryThread(QThread):
    progress = pyqtSignal(object)
    finished = pyqtSignal()

    def __init__(self, uav1):
        super().__init__()
        self.iha = uav1
        self.iha_bilgi = None
        self.iha_baglandi = False

    def run(self):
        global veriler, last_coor, durdur_btn_clicked, veri_yaz, destek_iha, destek_iha_baud, destek_iha_port,imha_iha, imha_iha_baud, imha_iha_port, baglanilan_iha, imulasyon_port, simulasyon_baud, simulasyon_iha
        veri_yaz = True
        try:
            # print(f'{ui.Ports.currentText().split("-")[0]}')
            if self.iha == 0:
                destek_iha = connect_iha(destek_iha_port, destek_iha_baud, destek_iha)
                baglanilan_iha = destek_iha
                while baglanilan_iha is None:
                    print("bekleniyor")
                    time.sleep(1)
                ui.mesajlarList.addItem(f"{dt.now().strftime('%H.%M.%S')} =>  Destek İHA'sına bağlanıldı.")
                self.iha_bilgi = destek_iha
                self.iha_baglandi = True

            elif self.iha == 1:
                imha_iha = connect_iha(imha_iha_port, imha_iha_baud, imha_iha)
                baglanilan_iha = imha_iha
                while baglanilan_iha is None:
                    print("bekleniyor")
                    time.sleep(1)
                ui.mesajlarList.addItem(f"{dt.now().strftime('%H.%M.%S')} =>  İmha İHA'sına bağlanıldı.")
                self.iha_bilgi = imha_iha
                self.iha_baglandi = True
            elif self.iha == 2:
                simulasyon_iha = connect_iha(simulasyon_port, simulasyon_baud, simulasyon_iha)
                baglanilan_iha = simulasyon_iha
                ui.mesajlarList.addItem(f"{dt.now().strftime('%H.%M.%S')} =>  Simülasyona bağlanıldı.")
                self.iha_bilgi = simulasyon_iha
                self.iha_baglandi = True
            else:
                print("iha bağlanmadı.")
            ui.mapThread.start()
            while veri_yaz and self.iha_baglandi:
                veriler = Telemetri(self.iha_bilgi)
                if veriler is not None:
                    try:
                        ui.paketNoValue.setText(str(veriler.paket_no))
                        ui.altValue.setText(str(veriler.alt))
                        ui.irtifaValue.setText(str(veriler.relative_alt))
                        ui.latValue.setText(str(veriler.lat))
                        ui.longValue.setText(str(veriler.long))
                        ui.modValue.setText(str(veriler.mod))
                        ui.sinyalGucValue.setText(str(veriler.sinyal))
                        armed = veriler.baseMod & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        if armed:
                            ui.armedDisarmedValue.setStyleSheet('color: green')
                            ui.armedDisarmedValue.setText(str("Armed"))
                        else:
                            ui.armedDisarmedValue.setStyleSheet('color: red')
                            ui.armedDisarmedValue.setText(str("Disarmed"))

                        try:
                            ui.mesafeValue.setText(str(haversine(hedef_coor[0], hedef_coor[1], veriler.lat, veriler.long)))
                        except Exception as err:
                            print(f"mesafe bilgisi yazdıralamadı. {err}")
                        try:
                            ui.servoDeger1.setText(str(getattr(veriler.signals, f'chan1_raw')))
                            ui.servoDeger2.setText(str(getattr(veriler.signals, f'chan2_raw')))
                            ui.servoDeger3.setText(str(getattr(veriler.signals, f'chan3_raw')))
                            ui.servoDeger4.setText(str(getattr(veriler.signals, f'chan4_raw')))
                            ui.servoDeger5.setText(str(getattr(veriler.signals, f'chan5_raw')))
                            ui.servoDeger6.setText(str(getattr(veriler.signals, f'chan6_raw')))
                            ui.servoDeger7.setText(str(getattr(veriler.signals, f'chan7_raw')))
                            ui.servoDeger8.setText(str(getattr(veriler.signals, f'chan8_raw')))
                            ui.servoDeger9.setText(str(getattr(veriler.signals, f'chan9_raw')))
                            ui.servoDeger10.setText(str(getattr(veriler.signals, f'chan10_raw')))
                            ui.servoDeger11.setText(str(getattr(veriler.signals, f'chan11_raw')))
                            ui.servoDeger12.setText(str(getattr(veriler.signals, f'chan12_raw')))
                            ui.servoDeger13.setText(str(getattr(veriler.signals, f'chan13_raw')))
                            ui.servoDeger14.setText(str(getattr(veriler.signals, f'chan14_raw')))
                        except Exception as err:
                            print(f"Servo bilgisi yazdıralamadı. {err}")
                        ui.rollSpeedValue.setText(str(veriler.roll_speed))

                        # göstergeler
                        try:
                            ui.adi.setRoll(float(veriler.roll))
                            ui.adi.setPitch(float(veriler.pitch))
                            ui.adi.viewUpdate.emit()

                            ui.alt.setAltitude(float(veriler.relative_alt * 3.28084))
                            ui.alt.viewUpdate.emit()

                            ui.hsi.setHeading(float(veriler.yaw))
                            ui.hsi.viewUpdate.emit()

                            ui.si.setSpeed(veriler.hava_hizi)
                            ui.si.viewUpdate.emit()

                        except Exception as e:
                            print(f"gosterge hata: {e}")


                    except Exception as err:
                        print(f"veri_yazdir fonksiyonu {err}")
            self.finished.emit()
            try:
                destek_iha.close()
                imha_iha.close()
            except Exception as e:
                #print(e)
                pass
            destek_iha = None
            imha_iha = None
        except Exception as err:
            print(f"TelemetryThread {err}")



#                                                                          * AUTONOMOUS FLIGHT CODE *
class DestekThread(QThread):
    progress = pyqtSignal(tuple)
    finished = pyqtSignal()

    def __init__(self, hedef_lat, hedef_lon):
        super().__init__()
        self.hedef_lat = hedef_lat
        self.hedef_lon = hedef_lon

    def run(self):
        # Destek İHA'sının hedefe gidiş uçuş kodu buraya gelecek

class DestekDonusThread(QThread):
    def run(self):
        # Destek İHA'sının eve dönüş uçuş kodu buraya gelecek

class ImhaThread(QThread):
    progress = pyqtSignal(tuple)
    finished = pyqtSignal()
    def __init__(self, ui, hedef_lat, hedef_lon):
        super().__init__()
        self.ui = ui
        self.hedef_lat = hedef_lat
        self.hedef_lon = hedef_lon

    def run(self):
        # İmha İHA'sının uçuş kodu buraya gelecek



is_finished = False
port_list = []


def port_guncelle():
    yeni_ports = serial.tools.list_ports.comports()
    try:
        for port in yeni_ports:
            if not str(port) in port_list:
                second_ui.destekPort.addItem(str(port).split(" -")[0])
                second_ui.imhaPort.addItem(str(port).split(" -")[0])
                port_list.append(str(port))
    except Exception as err:
        print(f"port_guncelle hata {err}")


def verileri_yazdir_btn():
    global imha_iha, destek_iha, destek_iha_connected, imha_iha_connected, baglanilan_iha, simulasyon_connected, iha_turu
    if str(ui.Ports.currentText().split("-")[0]) == "0 ":
        iha_turu = 1
        telemetry(0)
        destek_iha_connected = True

    elif str(ui.Ports.currentText().split("-")[0]) == "1 ":
        iha_turu = 2
        telemetry(1)
        imha_iha_connected = True

    elif str(ui.Ports.currentText().split("-")[0]) == "2 ":
        iha_turu = 1
        telemetry(2)
        simulasyon_connected = True



#                                                                          * CONTROL CENTER *
def make_iptal_btn_clicked_true():
    """iptal butonuna basıldığını kontrol eden fonksiyon"""
    global iptal_btn_clicked
    iptal_btn_clicked = True


def arm_et():
    global baglanilan_iha
    try:
        print(f"aaaaaaaaa: {type(destek_iha)}")

        if baglanilan_iha is not None:
            baglanilan_iha.mav.command_long_send(baglanilan_iha.target_system, baglanilan_iha.target_component,
                                        utility.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

        print("bbbbbbbbbb")
    except Exception as e:
        print(e)


def disarm_et():
    global baglanilan_iha
    try:
        if baglanilan_iha is not None:
            baglanilan_iha.mav.command_long_send(baglanilan_iha.target_system, baglanilan_iha.target_component,
                                        utility.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    except Exception as e:
        print(e)


def set_mode():
    global baglanilan_iha
    mode = str(ui.modes.currentText())
    baglanilan_iha.set_mode(mode)



#                                                                          * MAIN FUNCTION PROCESSING *
if __name__ == "__main__":
    import sys

    # Ayarlar için değişkenler
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    ui.settings = QSettings('YerIstasyonu', 'App1')


    value = 100
    # Göstergeler
    ui.adi = qfi_ADI.qfi_ADI(ui)
    ui.adi.resize(value, value)
    ui.adi.reinit()
    ui.adilayout.addWidget(ui.adi, 0, 0)

    ui.alt = qfi_ALT.qfi_ALT(ui)
    ui.alt.resize(value, value)
    ui.alt.reinit()
    ui.altlayout.addWidget(ui.alt, 0, 0)

    ui.si = qfi_SI.qfi_SI(ui)
    ui.si.resize(value, value)
    ui.si.reinit()
    ui.silayout.addWidget(ui.si, 0, 0)

    ui.hsi = qfi_HSI.qfi_HSI(ui)
    ui.hsi.resize(value, value)
    ui.hsi.reinit()
    ui.hsilayout.addWidget(ui.hsi, 0, 0)

    ui.armBtn.clicked.connect(arm_et)
    ui.armBtn.clicked.connect(disarm_et)
    ui.modAyarlaBtn.clicked.connect(set_mode)

    SecondWindow = QtWidgets.QMainWindow()
    second_ui = Ui_Settings()
    second_ui.setupUi(SecondWindow)
    port_guncelle()


    zamanlayici_counter = 5
    sunucu_yenileme = int(second_ui.sunucuGuncellemeSuresi.value())
    harita_yenileme = int(second_ui.haritaYenilenmeSuresi.value())


    def openSettings():
        SecondWindow.show()

    ui.ayarlarBtn.clicked.connect(openSettings)


    ui.iptalBtn.clicked.connect(make_iptal_btn_clicked_true)
    ui.mesajlarList.addItem(f"{dt.now().strftime('%H.%M.%S')} =>  Uygulama başlatıldı.")

    # Add the map to gui
    ui.my_map = Map()
    frame_layout = QVBoxLayout(ui.frame)
    ui.frame.setLayout(frame_layout)
    frame_layout.addWidget(ui.my_map)

    try:
        ui.mapThread = QThread()
        ui.mapWorker = (MapThread())
        ui.mapWorker.moveToThread(ui.mapThread)

        ui.mapWorker.finished.connect(ui.mapThread.quit)
        ui.mapWorker.finished.connect(ui.mapWorker.deleteLater)
        ui.mapThread.finished.connect(ui.mapThread.deleteLater)
        ui.mapWorker.progress.connect(map_timer.start)
        ui.mapThread.started.connect(ui.mapWorker.run)
    except Exception as err:
        print(f"MapThread oluşturma hatası.{err}")


    # Add the telemetry thread to qt
    def telemetry(uav1):
        try:
            Telemetri.paket_no = 0
            ui.telemetry_thread = QThread()
            ui.telemetry_worker = TelemetryThread(uav1)
            ui.telemetry_worker.moveToThread(ui.telemetry_thread)
            ui.telemetry_worker.finished.connect(ui.telemetry_thread.quit)
            ui.telemetry_worker.finished.connect(ui.telemetry_worker.deleteLater)
            ui.telemetry_thread.finished.connect(ui.telemetry_thread.deleteLater)
            ui.telemetry_thread.started.connect(ui.telemetry_worker.run)
            ui.yazdirBtn.setEnabled(False)
            ui.durdurBtn.setEnabled(True)
            ui.telemetry_thread.start()

        except Exception as err:
            print(f"Telemetri Thread oluşturma hatası {err}")


    ui.yazdirBtn.clicked.connect(verileri_yazdir_btn)
    ui.durdurBtn.clicked.connect(make_not_running)
    second_ui.yenileBtn.clicked.connect(port_guncelle)

    # Destek Thread
    def destek(latx, lonx):
        try:
            ui.destek_thread = QThread()
            ui.destek_worker = DestekThread(latx, lonx)
            ui.destek_worker.moveToThread(ui.destek_thread)
            ui.destek_worker.finished.connect(ui.destek_thread.quit)
            ui.destek_worker.finished.connect(ui.destek_worker.deleteLater)
            ui.destek_thread.finished.connect(ui.destek_thread.deleteLater)
            ui.destek_thread.started.connect(ui.destek_worker.run)
            ui.destek_thread.start()
        except Exception as err:
            print(f"Destek Thread oluşturma hatası {err}")

    def destek_donus():
        try:
            ui.destek_donus_thread = QThread()
            ui.destek_donus_worker = DestekDonusThread()
            ui.destek_donus_worker.moveToThread(ui.destek_donus_thread)
            ui.destek_donus_worker.finished.connect(ui.destek_donus_thread.quit)
            ui.destek_donus_worker.finished.connect(ui.destek_donus_worker.deleteLater)
            ui.destek_donus_thread.finished.connect(ui.destek_donus_thread.deleteLater)
            ui.destek_donus_thread.started.connect(ui.destek_donus_worker.run)
            ui.destek_donus_thread.start()
        except Exception as err:
            print(f"Destek Donus Thread oluşturma hatası {err}")

    def imha(latx, lony):
        try:
            ui.imha_thread = QThread()
            ui.imha_worker = ImhaThread(ui, latx, lony)
            ui.imha_worker.moveToThread(ui.imha_thread)
            # ui.imha_worker.progress.connect(iha_move)
            ui.imha_worker.finished.connect(ui.imha_thread.quit)
            ui.imha_worker.finished.connect(ui.imha_worker.deleteLater)
            ui.imha_thread.finished.connect(ui.imha_thread.deleteLater)
            ui.imha_thread.started.connect(ui.imha_worker.run)
            ui.imha_thread.start()
        except Exception as err:
            print(f"İmha Thread oluşturma hatası{err}")



    # Add the smartwatch messages thread to qt
    try:
        ui.messagesThread = QThread()
        ui.messagesWorker = MessagesThread()
        ui.messagesWorker.moveToThread(ui.messagesThread)

        ui.messagesWorker.finished.connect(ui.messagesThread.quit)
        ui.messagesWorker.finished.connect(ui.messagesWorker.deleteLater)
        ui.messagesThread.finished.connect(ui.messagesThread.deleteLater)
        ui.messagesWorker.progress.connect(akilli_saat_bilgilerini_yazdir)
        ui.messagesThread.started.connect(ui.messagesWorker.run)
        ui.messagesThread.start()

    except Exception as err:
        print(f"messagesThread oluşturma hatası {err}")

    MainWindow.show()

    sys.exit(app.exec_())
