import tkinter as tk
from tkinter import PhotoImage
import cv2
from PIL import Image, ImageTk
import os

root = tk.Tk()
root.title("Sterowanie Rumcajsem")
root.geometry("1400x900")
root.configure(bg="#ffffff")

cap = cv2.VideoCapture(2)  # ZED

# Ustawienia folderów
photos_dir = "photos_captured"
left_folder = os.path.join(photos_dir, "L")
right_folder = os.path.join(photos_dir, "R")
films_dir = "films_captured"

# Funkcje do zarządzania plikami
def create_folder_if_not_exists(folder):
    if not os.path.exists(folder):
        os.makedirs(folder)

def get_next_filename(folder, prefix, extension):
    files = [f for f in os.listdir(folder) if f.endswith(extension)]
    if files:
        max_num = max([int(f.split('_')[0]) for f in files])
        return f"{max_num + 1}_{prefix}.{extension}"
    else:
        return f"1_{prefix}.{extension}"

# Tworzenie folderów
create_folder_if_not_exists(left_folder)
create_folder_if_not_exists(right_folder)
create_folder_if_not_exists(films_dir)

# Funkcje do sterowania robotem i zapisywania plików
def turn_right():
    pass

def turn_left():
    pass

def go_straight():
    pass

def go_back():
    pass

def save_image():
    ret, frame = cap.read()
    if ret:
        height, width = frame.shape[:2]
        mid = width // 2
        left_frame = frame[:, :mid]
        right_frame = frame[:, mid:]
        
        # Zapis obrazów z podzielonego kadru
        filename_left = get_next_filename(left_folder, "L", "jpg")
        filename_right = get_next_filename(right_folder, "R", "jpg")
        cv2.imwrite(os.path.join(left_folder, filename_left), left_frame)
        cv2.imwrite(os.path.join(right_folder, filename_right), right_frame)
        print(f"Saved {filename_left} and {filename_right}")

def start_filming():
    global out_left, out_right, filming
    ret, frame = cap.read()
    if ret:
        height, width = frame.shape[:2]
        mid = width // 2

        # Przygotowanie do nagrywania
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        filename_left = get_next_filename(os.path.join(films_dir, "L"), "L", "avi")
        filename_right = get_next_filename(os.path.join(films_dir, "R"), "R", "avi")
        out_left = cv2.VideoWriter(os.path.join(films_dir, "L", filename_left), fourcc, 20.0, (mid, height))
        out_right = cv2.VideoWriter(os.path.join(films_dir, "R", filename_right), fourcc, 20.0, (mid, height))

        filming = True
        print(f"Started filming {filename_left} and {filename_right}")

def end_filming():
    global filming
    if filming:
        filming = False
        out_left.release()
        out_right.release()
        print("Stopped filming")

# Dodanie strzałek (należy zastąpić ścieżkami do odpowiednich obrazów)
arrow_left = PhotoImage(file="arrow_left.png")  
arrow_right = PhotoImage(file="arrow_right.png")  
arrow_up = PhotoImage(file="arrow_up.png")  
arrow_down = PhotoImage(file="arrow_down.png")  

# Układ przycisków
left_camera_label = tk.Label(root, bg="#4682B4")
left_camera_label.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky="nsew")

right_camera_label = tk.Label(root, bg="#4682B4")
right_camera_label.grid(row=0, column=2, columnspan=2, padx=5, pady=5, sticky="nsew")

start_filming_btn = tk.Button(root, text="START FILMING", command=start_filming, bg="#4682B4", fg="black", font=("Arial", 12))
start_filming_btn.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")

stop_filming_btn = tk.Button(root, text="STOP FILMING", command=end_filming, bg="#4682B4", fg="black", font=("Arial", 12))
stop_filming_btn.grid(row=1, column=1, padx=5, pady=5, sticky="nsew")

save_photo_btn = tk.Button(root, text="SAVE A PHOTO", command=save_image, bg="#4682B4", fg="black", font=("Arial", 12))
save_photo_btn.grid(row=1, column=2, columnspan = 2, padx=5, pady=5, sticky="nsew")

btn_left = tk.Button(root, image=arrow_left, command=turn_left, bg="#4682B4")
btn_left.grid(row=2, column=0, padx=5, pady=5, sticky="nsew")

btn_down = tk.Button(root, image=arrow_down, command=go_back, bg="#4682B4")
btn_down.grid(row=2, column=1, padx=5, pady=5, sticky="nsew")

btn_up = tk.Button(root, image=arrow_up, command=go_straight, bg="#4682B4")
btn_up.grid(row=2, column=2, padx=5, pady=5, sticky="nsew")

btn_right = tk.Button(root, image=arrow_right, command=turn_right, bg="#4682B4")
btn_right.grid(row=2, column=3, padx=5, pady=5, sticky="nsew")

# Funkcja zmieniająca rozmiar komponentów
def resize_components(event):
    root.grid_rowconfigure(0, weight=1)
    root.grid_rowconfigure(1, weight=1)
    root.grid_rowconfigure(2, weight=2)
    root.grid_columnconfigure(0, weight=1)
    root.grid_columnconfigure(1, weight=1)
    root.grid_columnconfigure(2, weight=1)
    root.grid_columnconfigure(3, weight=1)

root.bind('<Configure>', resize_components)

# Funkcja do aktualizacji obrazu z kamer
def update_frames():
    global filming
    ret, frame = cap.read()
    if ret:
        height, width = frame.shape[:2]
        mid = width // 2

        # Podział obrazu na lewą i prawą część
        left_frame = frame[:, :mid]
        right_frame = frame[:, mid:]
        
        # Aktualizacja lewego obrazu
        img_left = Image.fromarray(cv2.cvtColor(left_frame, cv2.COLOR_BGR2RGB))
        imgtk_left = ImageTk.PhotoImage(image=img_left)
        left_camera_label.imgtk = imgtk_left
        left_camera_label.configure(image=imgtk_left)

        # Aktualizacja prawego obrazu
        img_right = Image.fromarray(cv2.cvtColor(right_frame, cv2.COLOR_BGR2RGB))
        imgtk_right = ImageTk.PhotoImage(image=img_right)
        right_camera_label.imgtk = imgtk_right
        right_camera_label.configure(image=imgtk_right)

        # Zapis do plików wideo
        if filming:
            out_left.write(left_frame)
            out_right.write(right_frame)

    # Aktualizacja po 10 ms
    root.after(10, update_frames)

# Zmienna do kontrolowania stanu nagrywania
filming = False

# Inicjalne ustawienie rozmiaru komponentów
resize_components(None)

# Rozpoczęcie aktualizacji obrazu z kamer
update_frames()

root.mainloop()

# Zwolnienie zasobów kamery po zamknięciu aplikacji
cap.release()
cv2.destroyAllWindows()
