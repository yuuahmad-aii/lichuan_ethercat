# EtherCAT Servo Control GUI

Ini adalah aplikasi antarmuka grafis (GUI) sederhana yang dibuat dengan Python dan Tkinter untuk mengontrol dan memonitor sebuah servo drive yang kompatibel dengan profil CiA 402 melalui protokol EtherCAT.

## Fitur

- **Koneksi EtherCAT**: Terhubung dan terputus dari jaringan EtherCAT.
- **Manajemen Status Drive**:
  - Membaca dan menampilkan status drive sesuai standar CiA 402 (State, Status Word).
  - Melakukan reset pada kondisi *fault*.
  - Mengaktifkan drive (*Enable Operation*).
- **Monitoring Real-time**: Menampilkan posisi aktual, kecepatan aktual, dan mode operasi yang sedang berjalan.
- **Mode Operasi**:
  - **Profile Position Mode**: Mengirim drive ke posisi target absolut.
  - **Profile Velocity Mode**: Menjalankan drive pada kecepatan target.
- **Antarmuka User-Friendly**: Input yang jelas untuk nama antarmuka jaringan, posisi target, dan kecepatan target.

## Persyaratan

- **Python 3.x**
- **pysoem**: Pustaka Python untuk komunikasi EtherCAT.
- **Npcap** atau **WinPcap** (untuk pengguna Windows): Diperlukan oleh `pysoem` untuk mengakses *network interface* pada level *raw*.

## Instalasi

1.  **Instal Python**: Pastikan Python 3 sudah terinstal di sistem Anda.

2.  **Instal Npcap**: Unduh dan instal [Npcap](https://nmap.org/npcap/) untuk Windows. Selama instalasi, pastikan untuk mencentang opsi "Install Npcap in WinPcap API-compatible Mode".

3.  **Instal pysoem**: Buka terminal atau command prompt dan jalankan perintah berikut:
    ```sh
    pip install pysoem
    ```

## Penggunaan

1.  **Identifikasi Nama Network Interface**:
    - Skrip ini memerlukan nama *raw* dari *network interface* yang terhubung ke servo drive.
    - Anda dapat menemukan nama ini dengan menjalankan `pysoem.find_adapters()` atau melihat contoh yang diberikan di dalam GUI. Formatnya biasanya seperti `\Device\NPF_{GUID}`.

2.  **Jalankan Skrip**:
    - **PENTING**: Jalankan skrip ini dengan hak akses administrator (klik kanan -> "Run as administrator") agar dapat mengakses *network interface*.
    ```sh
    python servo_control_gui.py
    ```

3.  **Langkah-langkah di GUI**:
    - **Masukkan Nama Interface**: Salin dan tempel nama *network interface* Anda ke dalam kolom yang tersedia.
    - **Hubungkan**: Klik tombol **Connect**. Status akan berubah menjadi "Connected" jika berhasil menemukan satu atau lebih *slave*.
    - **Aktifkan Drive**:
      - Jika drive dalam kondisi *fault*, klik **Reset Fault**.
      - Klik **Enable Drive** untuk menjalankan urutan pengaktifan status CiA 402 (Shutdown -> Switch On -> Enable Operation). Tunggu hingga status berubah menjadi "Operation Enabled".
    - **Pilih Mode**: Pilih antara "Position Mode" atau "Velocity Mode".
    - **Kirim Perintah**:
      - Untuk **Position Mode**: Masukkan posisi target, lalu klik **Move**.
      - Untuk **Velocity Mode**: Masukkan kecepatan target, lalu klik **Run**.
    - **Putuskan Koneksi**: Klik **Disconnect** untuk menghentikan komunikasi dan menonaktifkan drive.

## Catatan

- Skrip ini secara *default* dikonfigurasi untuk berkomunikasi dengan **satu servo drive** (slave pertama yang ditemukan di jaringan EtherCAT).
- Pastikan koneksi fisik antara PC dan servo drive sudah benar.
- Nilai PDO (Process Data Object) yang di-map dalam kode mungkin perlu disesuaikan tergantung pada konfigurasi spesifik dari servo drive Anda.
