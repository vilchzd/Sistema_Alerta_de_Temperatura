/*
Copyright 2020 Gilbert François Duivesteijn
Modified 2023 Diego Vilchez Villalobos

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include "ThermalCamera.h"
#include "constants.h"
#include "colormap.h"
#include <ctime>
#include <pigpio.h>
#include <unistd.h>
#include <thread>
#include <vector>
#include <boost/bind/bind.hpp>
#include <filesystem>
#include <experimental/filesystem>
using namespace boost::placeholders;
using namespace std;
using namespace TgBot;

#define BUZZER_PIN 18
//Token de bot de Telegram
const std::string token = "Your token";
//Chat IDS para recibir alertas
vector<std::string> chatIds = {"1584228134as", "987654321xy", "246813579zw"}; //ChatIDS



ThermalCamera::ThermalCamera() {
    last_screenshot_time = std::chrono::steady_clock::now();
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "=== ThermalCamera, Copyright 2020 Ava-X ===");
    char *base_path = SDL_GetBasePath();
    if (base_path) {
        resource_path = std::string(base_path) + "../resources";
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Resource path: %s\n", resource_path.c_str());
    }
    init_sdl();
    init_sensor();
    is_running = true;
    is_measuring = false;
    is_measuring_lpf = is_measuring;
    mean_temp = 0.0f;
    mean_temp_lpf = 0.0f;
    timer_is_animating = 0;
    animation_frame_nr = 0;
    frame_no = 0;

}

ThermalCamera::~ThermalCamera() {
    clean();
}

void ThermalCamera::init_sdl() {
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_Init() Failed: %s\n", SDL_GetError());
    }
    window = SDL_CreateWindow("ThermalCamera", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 480, 800,0);
    //window = SDL_CreateWindow("ThermalCamera", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,0,0,SDL_WINDOW_FULLSCREEN);
    if (window == nullptr) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_CreateWindow() Failed: %s\n", SDL_GetError());
        clean();
        exit(EXIT_FAILURE);
    }
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (renderer == nullptr) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_CreateRenderer() Failed: %s\n", SDL_GetError());
        clean();
        exit(EXIT_FAILURE);
    }
    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, SENSOR_W, SENSOR_H);
    if (texture == nullptr) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_CreateTexture() Failed: %s\n", SDL_GetError());
        clean();
        exit(EXIT_FAILURE);
    }

    texture_r = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_TARGET, SENSOR_H, SENSOR_H);
    if (texture_r == nullptr) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_CreateTexture() Failed: %s\n", SDL_GetError());
        clean();
        exit(EXIT_FAILURE);
    }
    // Load and create slider background
    std::string slider_bg_path = resource_path + "/images/slider_bg.bmp";
    SDL_Surface *image = SDL_LoadBMP(slider_bg_path.c_str());
    if (image == nullptr) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_LoadBMP() Failed: %s\n", SDL_GetError());
        clean();
        exit(EXIT_FAILURE);
    }
    slider = SDL_CreateTextureFromSurface(renderer, image);
    SDL_FreeSurface(image);
    // Load animation
    for (int i = 0; i < 6; i++) {
        std::string file_path = resource_path + "/images/anim" + std::to_string(i + 1) + ".bmp";
        SDL_Surface *_image = SDL_LoadBMP(file_path.c_str());
        if (_image == nullptr) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_LoadBMP() Failed: %s\n", SDL_GetError());
            clean();
            exit(EXIT_FAILURE);
        }
        SDL_Texture *_frame = SDL_CreateTextureFromSurface(renderer, _image);
        animation.push_back(_frame);
    }
    // Hide cursor
    SDL_ShowCursor(SDL_DISABLE);
    // Init fonts
    TTF_Init();
    font64 = TTF_OpenFont(FONT_PATH.c_str(), 64);
    font32 = TTF_OpenFont(FONT_PATH.c_str(), 36);
    if (font64 == nullptr) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Unable to load font %s", FONT_PATH.c_str(), TTF_GetError());
        clean();
        exit(EXIT_FAILURE);
    }
    SDL_GetRendererOutputSize(renderer, &display_width, &display_height);
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Display dimension: (%d, %d)", display_width, display_height);
    // Set scaling and aspect ratio
    const double display_ratio = (double) display_width / display_height;
    const double sensor_ratio = (double) SENSOR_W / SENSOR_H;
    if (display_ratio >= sensor_ratio) {
        aspect_scale = display_height / SENSOR_H;
    } else {
        aspect_scale = display_width / SENSOR_W;
    }
    output_width = SENSOR_W * aspect_scale;
    output_height = SENSOR_H * aspect_scale;
    offset_left = (display_width - output_width) / 2;
    offset_top = (display_height - output_height) / 2;
    // Override offset top to align the image with the top edge.
    offset_top = 0;
    rect_preserve_aspect = (SDL_Rect) {.x = offset_left, .y = offset_top, .w = output_width, .h = output_height};
    rect_fullscreen = (SDL_Rect) {.x = 0, .y = 0, .w = display_width, .h = display_height};
}

void ThermalCamera::init_sensor() {
    MLX90640_SetDeviceMode(MLX_I2C_ADDR, 0);
    MLX90640_SetSubPageRepeat(MLX_I2C_ADDR, 0);
    switch (FPS) {
        case 1:
            MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b001);
            break;
        case 2:
            MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b010);
            break;
        case 4:
            MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b011);
            break;
        case 8:
            MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b100);
            break;
        case 16:
            MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b101);
            break;
        case 32:
            MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b110);
            break;
        case 64:
            MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b111);
            break;
        default:
            printf("Unsupported framerate: %d", FPS);
            clean();
            exit(EXIT_FAILURE);
    }
    MLX90640_SetChessMode(MLX_I2C_ADDR);
    MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
    MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
}

void ThermalCamera::clean() {
    if (window != nullptr) {
        SDL_DestroyWindow(window);
    }
    if (renderer != nullptr) {
        SDL_DestroyRenderer(renderer);
    }
    if (texture != nullptr) {
        SDL_DestroyTexture(texture);
    }
    if (texture_r != nullptr) {
        SDL_DestroyTexture(texture_r);
    }
    SDL_Quit();
}

void ThermalCamera::update() {
    frame_no++;
    auto start = std::chrono::system_clock::now();
    MLX90640_GetFrameData(MLX_I2C_ADDR, frame);

    eTa = MLX90640_GetTa(frame, &mlx90640) - 6.0f;
    MLX90640_CalculateTo(frame, &mlx90640, EMISSIVITY, eTa, mlx90640To);

    MLX90640_BadPixelsCorrection((&mlx90640)->brokenPixels, mlx90640To, 1, &mlx90640);
    MLX90640_BadPixelsCorrection((&mlx90640)->outlierPixels, mlx90640To, 1, &mlx90640);

    // Scan the sensor and compute the mean skin temperature, assuming that skin temperature is between
    // MIN_MEASURE_RANGE and MAX_MEASURE_RANGE.
    float sum_temp = 0.0f;
    int n_samples = 0;
    for (int y = 0; y < SENSOR_W; y++) {
        for (int x = 0; x < SENSOR_H; x++) {
            // Read the temperature value for this pixel.
            float val = mlx90640To[SENSOR_H * (SENSOR_W - 1 - y) + x];
            // Map the value to a color
            colormap(y, x, val, MIN_COLORMAP_RANGE, MAX_COLORMAP_RANGE);
            // Sum and count the temperatures within the skin temperature range.
            if (val > MIN_MEASURE_RANGE && val < MAX_MEASURE_RANGE) {
                sum_temp += val;
                n_samples += 1;
            }
        }
    }
    // Check if there are enough pixels within the temperature measuring range.
    bool is_measuring_prev = is_measuring;
    is_measuring = n_samples > MEASURE_AREA_THRESHOLD;
    if (is_measuring_prev != is_measuring) {
        timer_is_measuring = 0;
    } else {
        timer_is_measuring++;
    }
    if (timer_is_measuring > TIMER_THRESHOLD_FRAMES) {
        is_measuring_lpf = is_measuring;
    }
    // Compute the mean of the temperatures in the range.
    if (n_samples > 0) {
        mean_temp = sum_temp / (float) n_samples;
    } else {
        mean_temp = -1.0f;
    }
    // Smooth the mean temperature over time (moving mean), because the sensor is a bit noisy.
    if (mean_temp_lpf > 0 && mean_temp > MIN_MEASURE_RANGE && mean_temp < MAX_MEASURE_RANGE) {
        // Use moving mean only if the difference between current temp and mean_temp is not too large.
        if (abs(mean_temp_lpf - mean_temp) < 0.6) {
            mean_temp_lpf = BETA * mean_temp_lpf + (1 - BETA) * mean_temp;
        } else {
            mean_temp_lpf = mean_temp;
        }
    } else if (mean_temp_lpf < 0 && mean_temp > MIN_MEASURE_RANGE && mean_temp < MAX_MEASURE_RANGE) {
        mean_temp_lpf = mean_temp;
    } else {
        mean_temp_lpf = -1.0f;
    }
    // Format the temperature value to string
    std::stringstream message_ss;
    if (mean_temp > MIN_MEASURE_RANGE && mean_temp < MAX_MEASURE_RANGE) {
        message_ss << std::fixed << std::setprecision(1) << std::setw(4);
        message_ss << mean_temp_lpf + 5.6 << "\xB0" << "C" << std::endl; //Ajuste
        message = message_ss.str();
    } else {
        message = "";
    }
}


void ThermalCamera::render() {

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    render_sensor_frame();
    if (is_measuring_lpf) {
        render_slider();
        render_temp_labels();
    } else {
        render_animation();
    }
    SDL_RenderPresent(renderer);
    //screenshot();
}

void ThermalCamera::soundBuzzer() const {
    std::thread buzzerThread([this]() {
        if (gpioInitialise() < 0) {
            std::cout << "Failed to initialize pigpio library." << std::endl;
        }

        // Set up buzzer pin as output
        gpioSetMode(BUZZER_PIN, PI_OUTPUT);

        try {
            // Loop 2 times
            for (int i = 0; i < 2; i++) {
                gpioWrite(BUZZER_PIN, 1);
                usleep(50000);
                gpioWrite(BUZZER_PIN, 0);
                usleep(50000);
                gpioWrite(BUZZER_PIN, 1);
                usleep(50000);
                gpioWrite(BUZZER_PIN, 0);
                usleep(50000);
                gpioWrite(BUZZER_PIN, 1);
                usleep(50000);
                gpioWrite(BUZZER_PIN, 0);
                usleep(50000);
                gpioWrite(BUZZER_PIN, 1);
                usleep(50000);
                gpioWrite(BUZZER_PIN, 0);
                usleep(1000000);
            }
        } catch (...) {
            // Handle any exceptions here
        }

        // Cleanup pigpio library
        gpioTerminate();
    });

    buzzerThread.detach(); // Detach the thread to run independently
}

string ThermalCamera::tiempoActual() const{
    // Get the current time
    time_t currentTime = time(nullptr);
    // Convert the current time to local time
    std::tm* localTime = localtime(&currentTime);
    // Format and print the local time
    std::stringstream ss;
    ss << put_time(localTime, "%H:%M:%S %d/%m/%Y");
    // Return the formatted time as a string
    return ss.str();
}

string ThermalCamera::tiempolocal_H() const{
    time_t currentTime = time(nullptr);
    std::tm* localTime = localtime(&currentTime);
    std::stringstream ss;
    ss << put_time(localTime, "%d%m%y");
    return ss.str();
}

string ThermalCamera::tiempolocal_M() const{
    time_t currentTime = time(nullptr);
    std::tm* localTime = localtime(&currentTime);
    std::stringstream ss;
    ss << std::put_time(localTime, "%m%y"); // Change the format to only include month and year
    return ss.str();
}

string ThermalCamera::getLatestFile(const string& folderPath) const{
    string latestFile;
    filesystem::file_time_type latestTime;
    for (const auto& entry : filesystem::directory_iterator(folderPath)) {
        if (filesystem::is_regular_file(entry) && entry.path().extension() == ".bmp") {
            string fileName = entry.path().stem().string(); // Get the file name without extension
            if (fileName.size() >= 15 && fileName.substr(0, 6) == tiempolocal_H()) {
                string fileTimeStr = fileName.substr(7); // Extract the time part of the file name
                tm fileTime = {};
                istringstream ss(fileTimeStr);
                ss >> get_time(&fileTime, "%H.%M");
                if (!ss.fail()) {
                    filesystem::file_time_type fileTimeStamp = filesystem::last_write_time(entry);
                    if (latestFile.empty() || fileTimeStamp > latestTime) {
                        latestTime = fileTimeStamp;
                        latestFile = entry.path().string();
                    }
                }
            }
        }
    }

    return latestFile;
}

std::string ThermalCamera::alertaDate(const std::string& filePath) const {
    std::string fileName = std::filesystem::path(filePath).filename().string(); // Get the file name from the file path
    std::string dateTimePart = fileName.substr(0, 6); // Extract the date part from the file name
    std::string timePart = fileName.substr(7, 8); // Extract the time part from the file name
    std::replace(timePart.begin(), timePart.end(), '.', ':');
    dateTimePart.insert(2, "/"); // Insert a slash (/) at position 2
    dateTimePart.insert(5, "/"); // Insert a slash (/) at position 5
    dateTimePart += " " + timePart; // Append the time part
    return dateTimePart;
}

//Ajustar Lecturas de temperatura
void ThermalCamera::render_temp_labels() const {

    SDL_Point origin = {0, 640};
    SDL_Color text_color = {255, 255, 255, 255};
    
    std::string label = "Temperatura corporal:";
    std::string titulo = "Esperando personas:";
    
    if (mean_temp_lpf + 5.6 <= 32.0) { 
            render_text(message, {0, 0, 0, 255}, origin, 1, font32);
            render_text(label, {0, 0, 0, 255}, origin, 0, font32);
            render_text(titulo, {255, 255, 255, 255}, origin, 0, font32);
    } else {
            render_text(message, {255, 255, 255, 255}, origin, 1, font32);
            render_text(titulo, {0, 0, 0, 255}, origin, 0, font32);
            render_text(label, {255, 255, 255, 255}, origin, 0, font32);
    }

    //
    //origin.y -= 30;
    //
    origin = {0, 0};
    
    if (mean_temp_lpf + 5.6 <= 33.0) {
        label = "Baja";
    } else if (mean_temp_lpf + 5.6 > 33.0 && mean_temp_lpf + 5.6 <= 36.2) {
        label = "Normal";
    } else if (mean_temp_lpf + 5.6 > 36.2 && mean_temp_lpf + 5.6  <= 37.5) {
        label = "Alta";
    } else if (mean_temp_lpf + 5.6 > 37.5) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_screenshot_time);
        if (elapsed_time >= std::chrono::seconds(5)) {
            label = "Muy alta";
            render_text(label, text_color, origin, 3, font64);
            soundBuzzer();
            screenshot();
            last_screenshot_time = current_time;
            std::thread alertaThread([this]() {
                const string folderFilePath = "/home/electronica/Pictures";
                const string photoMimeType = "image/bmp";
                TgBot::Bot bot(token);
                for (const std::string& chatId : chatIds) {
                bot.getApi().sendPhoto(chatId, TgBot::InputFile::fromFile(getLatestFile(folderFilePath), photoMimeType));
                string tiempoAct = "¡Alerta temperatura alta!\nRegistrada a las: " + tiempoActual();
                bot.getApi().sendMessage(chatId, tiempoAct);
            }
            });
            alertaThread.detach();
            
        } else {
            label = "Muy alta (Alerta Enviada)";
        }
    }
    render_text(label, text_color, origin, 3, font64); //This one
}

void ThermalCamera::render_text(const std::string &text, const SDL_Color &text_color, const SDL_Point origin,
                           const int anchor,
                           TTF_Font *font) const {
    SDL_Surface *surf = TTF_RenderText_Solid(font, text.c_str(), text_color);
    SDL_Texture *texture_txt = SDL_CreateTextureFromSurface(renderer, surf);
    int text_width, text_height;
    SDL_QueryTexture(texture_txt, nullptr, nullptr, &text_width, &text_height);
    SDL_Rect dst;
    // top left
    if (anchor == 0) {
        dst = {origin.x, origin.y, text_width, text_height};
    }
        // top right
    else if (anchor == 1) {
        dst = {display_width - text_width - origin.x, origin.y, text_width, text_height};
    }
        // bottom right
    else if (anchor == 2) {
        dst = {display_width - text_width - origin.x, display_height - text_height - origin.y, text_width,
               text_height};
    }
        // bottom left
    else if (anchor == 3) {
        dst = {origin.x, display_height - text_height - origin.y, text_width, text_height};
    }
    SDL_RenderCopy(renderer, texture_txt, nullptr, &dst);
    SDL_FreeSurface(surf);
    SDL_DestroyTexture(texture_txt);
}

void ThermalCamera::render_sensor_frame() const {
    SDL_UpdateTexture(texture, nullptr, (uint8_t *) pixels, SENSOR_W * sizeof(uint32_t));
    SDL_SetRenderTarget(renderer, texture_r);
    SDL_RenderCopyEx(renderer, texture, nullptr, nullptr, rotation, nullptr, SDL_FLIP_NONE);
    SDL_SetRenderTarget(renderer, nullptr);
    if (preserve_aspect) {
        SDL_RenderCopy(renderer, texture_r, nullptr, &rect_preserve_aspect);
    } else {
        SDL_RenderCopy(renderer, texture_r, nullptr, &rect_fullscreen);
    }
}

void ThermalCamera::render_slider() const {
    if (!is_measuring_lpf) {
        return;
    }
    const int ys1 = 670 + 20;
    const int ys2 = 670 + 60;
    const int margin1 = 4;
    const int margin2 = margin1 + 2;
    SDL_Rect marker_rect;
    // slider background
    SDL_Rect rect_slider = {0, ys1, display_width, ys2 - ys1};
    SDL_RenderCopy(renderer, slider, nullptr, &rect_slider);
    // marker
    auto x_pos = (mean_temp_lpf - 31+6) / (39 - 31);
    x_pos = fmin(1.0, x_pos);
    x_pos = fmax(0.0, x_pos);
    int x_marker = static_cast<int>(round(x_pos * (float) display_width));

    marker_rect = {x_marker - margin2, ys1 - margin2, ys2 - ys1 + 2 * margin2};
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderFillRect(renderer, &marker_rect);

    marker_rect = {x_marker - margin1, ys1 - margin1, 2 * margin1, ys2 - ys1 + 2 * margin1};
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderFillRect(renderer, &marker_rect);

}

void ThermalCamera::handle_events() {
    SDL_Event event;
    SDL_PollEvent(&event);
    if (event.type == SDL_QUIT) {
        is_running = false;
    }
    if (event.type == SDL_KEYDOWN) {
        switch (event.key.keysym.sym) {
            case SDLK_ESCAPE:
                is_running = false;
                break;
            default:
                break;
        }
    }
}

void ThermalCamera::colormap(const int x, const int y, float v, float vmin, float vmax) {

    // Normalize v
    v = (v - vmin) / (vmax - vmin);
    auto color_index = static_cast<size_t>(round(255 * v));
    color_index = color_index > 255 ? 255 : color_index;
    color_index = color_index < 0 ? 0 : color_index;
    const uint offset = (y * SENSOR_W + x);
    ColorMap cm = get_colormap_jet();
    pixels[offset] = cm.b.at(color_index) << 16u | cm.g.at(color_index) << 8u | cm.r.at(color_index);
}

void ThermalCamera::render_animation() {

    if (timer_is_animating > TIMER_THRESHOLD_FRAMES) {
        timer_is_animating = 0;
        animation_frame_nr++;
        animation_frame_nr = animation_frame_nr >= animation.size() ? 0 : animation_frame_nr;
    } else {
        timer_is_animating++;
    }
    SDL_Rect animation_rect = {0, output_height, display_width, display_height - output_height};
    SDL_RenderCopy(renderer, animation.at(animation_frame_nr), nullptr, &animation_rect);

}

void ThermalCamera::screenshot() const{
    SDL_Surface *sshot = SDL_CreateRGBSurface(0, display_width, display_height, 32, 0x00ff0000, 0x0000ff00, 0x000000ff,
                                              0xff000000);
    SDL_RenderReadPixels(renderer, nullptr, SDL_PIXELFORMAT_ARGB8888, sshot->pixels, sshot->pitch);
    std::time_t currentTime = std::time(nullptr);
    std::tm* localTime = std::localtime(&currentTime);
    std::stringstream ss;
    ss << std::put_time(localTime,"%d%m%y_%H.%M.%S");
    //ss << std::setfill('0') << std::setw(5) << frame_no;
    auto filename = "/home/electronica/Pictures/" + ss.str() + ".bmp";
    SDL_SaveBMP(sshot, filename.c_str());
    SDL_FreeSurface(sshot);
}

// ----------------------------------------------------------------------Integracion del bot de telegram-------------------------------------------------------------------//
void ThermalCamera::runbot() {
    
    TgBot::Bot bot(token);
    
    const string folderFilePath = "/home/electronica/Pictures";
    const string photoMimeType = "image/bmp";
    
    bot.getEvents().onCommand("start", [this, &bot](TgBot::Message::Ptr message) {
        bot.getApi().sendMessage(message->chat->id, "Bot se encuentra enlazado");
    });
    
    bot.getEvents().onCommand("tiempo", [this, &bot](TgBot::Message::Ptr message) {
        string tiempoAct = "El tiempo actual es: " + tiempoActual();
        bot.getApi().sendMessage(message->chat->id, tiempoAct);
    });
/*
    bot.getEvents().onAnyMessage([this, &bot](TgBot::Message::Ptr message) {
        printf("User wrote %s\n", message->text.c_str());
        if (StringTools::startsWith(message->text, "/start")) {
            return;
        }
        bot.getApi().sendMessage(message->chat->id, "Your message is: " + message->text);
    });
*/

    bot.getEvents().onAnyMessage([this, &bot](TgBot::Message::Ptr message) {
        printf("User wrote %s\n", message->text.c_str());

        if (StringTools::startsWith(message->text, "/start") ||
        message->text == "/menu" ||
        message->text == "/photo" ||
        message->text == "/folder" ||
        message->text == "/alertahoy" ||
        message->text == "/alertames" ||
        message->text == "/alertault" ||
        message->text == "/tiempo") {
        return;
        }
        bot.getApi().sendMessage(message->chat->id, "Por favor utiliza el comando /menu para desplegar las opciones disponibles de la aplicación");
    });
    
    //---------------------------------------------Alertas--------------------------------------------------------------
    {
    //TODOS
    bot.getEvents().onCommand("folder", [this, &bot, &folderFilePath, &photoMimeType](TgBot::Message::Ptr message) {
        for (const auto& entry : std::filesystem::directory_iterator(folderFilePath)) {
            if (std::filesystem::is_regular_file(entry) && entry.path().extension() == ".bmp") {
                const string contenidoFilePath = entry.path().string();
                string alertaEnv = alertaDate(contenidoFilePath);
                bot.getApi().sendPhoto(message->chat->id, TgBot::InputFile::fromFile(contenidoFilePath, photoMimeType));
                bot.getApi().sendMessage(message->chat->id, "Alerta enviada el: "+alertaEnv);
            } else {
            bot.getApi().sendMessage(message->chat->id, "No hay alertas registradas.");
            }
        }
        bot.getApi().sendMessage(message->chat->id, "Fin de todas las alertas");
    });
    //MENSUA
    bot.getEvents().onCommand("alertames", [this, &bot, &folderFilePath, &photoMimeType](TgBot::Message::Ptr message) {
        std::string currentMonth = tiempolocal_M(); // Get the current month and year
        bool foundFiles = false;
        for (const auto& entry : std::filesystem::directory_iterator(folderFilePath)) {
            if (std::filesystem::is_regular_file(entry) && entry.path().extension() == ".bmp") {
                std::string fileName = entry.path().stem().string(); // Get the file name without extension
                if (fileName.substr(2, 4) == currentMonth) { // Check if the month part matches the current month
                    const std::string contenidoFilePath = entry.path().string();
                    string alertaEnv = alertaDate(contenidoFilePath);
                    bot.getApi().sendPhoto(message->chat->id, TgBot::InputFile::fromFile(contenidoFilePath, photoMimeType));
                    bot.getApi().sendMessage(message->chat->id, "Alerta enviada el: "+alertaEnv);
                    foundFiles = true; // Set the flag to true if at least one file is found
                }
            }
        }
        
        bot.getApi().sendMessage(message->chat->id, "Fin de alertas mensuales");

        if (!foundFiles) {
            bot.getApi().sendMessage(message->chat->id, "No se encuentran alertas este mes.");
        }
    });

    //DIARIO
    bot.getEvents().onCommand("alertahoy", [this, &bot, &folderFilePath, &photoMimeType](TgBot::Message::Ptr message) {
        std::string currentDate = tiempolocal_H();
        bool foundFiles = false;

        for (const auto& entry : std::filesystem::directory_iterator(folderFilePath)) {
            if (std::filesystem::is_regular_file(entry) && entry.path().extension() == ".bmp") {
                std::string fileName = entry.path().stem().string(); // Get the file name without extension
                if (fileName.substr(0, 6) == currentDate) { // Check if the date part matches the current date
                    const std::string contenidoFilePath = entry.path().string();
                    string alertaEnv = alertaDate(contenidoFilePath);
                    bot.getApi().sendPhoto(message->chat->id, TgBot::InputFile::fromFile(contenidoFilePath, photoMimeType));
                    bot.getApi().sendMessage(message->chat->id, "Alerta enviada el: "+alertaEnv);
                    foundFiles = true; // Set the flag to true if at least one file is found
                }
            }
        }
        
        bot.getApi().sendMessage(message->chat->id, "Fin de alertas diarias");

        if (!foundFiles) {
            bot.getApi().sendMessage(message->chat->id, "No se encuentran alertas el día de hoy.");
        }
    });

    //ULTIMA
    bot.getEvents().onCommand("alertault", [this, &bot, &folderFilePath, &photoMimeType](TgBot::Message::Ptr message) {
        string latestFile = getLatestFile(folderFilePath);
        string alertaEnv = alertaDate(latestFile);
        if (!latestFile.empty()) {
            bot.getApi().sendPhoto(message->chat->id, TgBot::InputFile::fromFile(latestFile, photoMimeType));
            bot.getApi().sendMessage(message->chat->id, "Alerta enviada el: "+alertaEnv);
        } else {
            bot.getApi().sendMessage(message->chat->id, "No hay alertas registradas.");
        }
    });
    }

    //-------------------------------------------------------Botones--------------------------------------------------
    {
    bot.getEvents().onCommand("menu", [this, &bot](TgBot::Message::Ptr message) {
        // Create an inline keyboard markup...
        TgBot::InlineKeyboardMarkup::Ptr keyboard(new TgBot::InlineKeyboardMarkup);

        //button 1
        TgBot::InlineKeyboardButton::Ptr button1(new TgBot::InlineKeyboardButton);
        button1->text = "Verificar enlace";
        button1->callbackData = "button1";  // Callback data to identify the button
        //button 2
        TgBot::InlineKeyboardButton::Ptr button2(new TgBot::InlineKeyboardButton);
        button2->text = "Tiempo local";
        button2->callbackData = "button2";  
        //button 3
        TgBot::InlineKeyboardButton::Ptr button3(new TgBot::InlineKeyboardButton);
        button3->text = "Última Alerta";
        button3->callbackData = "button3";  
        //button 4
        TgBot::InlineKeyboardButton::Ptr button4(new TgBot::InlineKeyboardButton);
        button4->text = "Alertas Diarias";
        button4->callbackData = "button4";  
        //button 5
        TgBot::InlineKeyboardButton::Ptr button5(new TgBot::InlineKeyboardButton);
        button5->text = "Alertas Mensuales";
        button5->callbackData = "button5"; 
        //button 6
        TgBot::InlineKeyboardButton::Ptr button6(new TgBot::InlineKeyboardButton);
        button6->text = "Todas Alertas";
        button6->callbackData = "button6";   
        // Create a vector to hold the rows of buttons
        std::vector<TgBot::InlineKeyboardButton::Ptr> row1;
        std::vector<TgBot::InlineKeyboardButton::Ptr> row2;
        // Add the button to the row
        row1.push_back(button1);
        row1.push_back(button2);
        row1.push_back(button3);
        row2.push_back(button4);
        row2.push_back(button5);
        row2.push_back(button6);

        // Add the row to the keyboard
        keyboard->inlineKeyboard.push_back(row1);
        keyboard->inlineKeyboard.push_back(row2);

        // Create a message with the keyboard
        std::string text = "Elige un botón para realizar una acción:";
        bot.getApi().sendMessage(message->chat->id, text, false, 0, keyboard);
        });

        // Add the onCallbackQuery event handler here
        bot.getEvents().onCallbackQuery([this, &bot](TgBot::CallbackQuery::Ptr query) {
        std::string callbackData = query->data;
        //Paths
        
        const string photoMimeType = "image/bmp";
        const string folderFilePath = "/home/electronica/Pictures";
        
        // Check the callback data to identify the button clicked
        if (callbackData == "button1") {
        // Handle button 1 click
            bot.getApi().sendMessage(query->message->chat->id, "El sistema se encuentra enlazado adecuadamente");

        } else if (callbackData == "button2") {
        // Handle button 2 click
            string tiempoAct = "El tiempo actual del sistema es: " + tiempoActual();
            bot.getApi().sendMessage(query->message->chat->id, tiempoAct);

        } else if (callbackData == "button3") {
        // Handle button 3 click
            string latestFile = getLatestFile(folderFilePath);
            string alertaEnv = alertaDate(latestFile);
            if (!latestFile.empty()) {
                bot.getApi().sendPhoto(query->message->chat->id, TgBot::InputFile::fromFile(latestFile, photoMimeType));
                bot.getApi().sendMessage(query->message->chat->id, "Alerta enviada el: "+alertaEnv);
            } else {
                bot.getApi().sendMessage(query->message->chat->id, "No hay alertas registradas.");
            }



        } else if (callbackData == "button4") {
        // Handle button 4 click
            std::string currentDate = tiempolocal_H();
            bool foundFiles = false;

            for (const auto& entry : std::filesystem::directory_iterator(folderFilePath)) {
                if (std::filesystem::is_regular_file(entry) && entry.path().extension() == ".bmp") {
                    std::string fileName = entry.path().stem().string(); // Get the file name without extension
                    if (fileName.substr(0, 6) == currentDate) { // Check if the date part matches the current date
                        const std::string contenidoFilePath = entry.path().string();
                        string alertaEnv = alertaDate(contenidoFilePath);
                        bot.getApi().sendPhoto(query->message->chat->id, TgBot::InputFile::fromFile(contenidoFilePath, photoMimeType));
                        bot.getApi().sendMessage(query->message->chat->id, "Alerta enviada el: "+alertaEnv);
                        foundFiles = true; // Set the flag to true if at least one file is found
                    }
                }
            }
            
            bot.getApi().sendMessage(query->message->chat->id, "Fin de alertas diarias");

            if (!foundFiles) {
                bot.getApi().sendMessage(query->message->chat->id, "No se encuentran alertas el día de hoy.");
            }



        } else if (callbackData == "button5") {
        // Handle button 5 click
            std::string currentMonth = tiempolocal_M(); // Get the current month and year
            bool foundFiles = false;
            for (const auto& entry : std::filesystem::directory_iterator(folderFilePath)) {
                if (std::filesystem::is_regular_file(entry) && entry.path().extension() == ".bmp") {
                    std::string fileName = entry.path().stem().string(); // Get the file name without extension
                    if (fileName.substr(2, 4) == currentMonth) { // Check if the month part matches the current month
                        const std::string contenidoFilePath = entry.path().string();
                        string alertaEnv = alertaDate(contenidoFilePath);
                        bot.getApi().sendPhoto(query->message->chat->id, TgBot::InputFile::fromFile(contenidoFilePath, photoMimeType));
                        bot.getApi().sendMessage(query->message->chat->id, "Alerta enviada el: "+alertaEnv);
                        foundFiles = true; // Set the flag to true if at least one file is found
                    }
                }
            }
            
            bot.getApi().sendMessage(query->message->chat->id, "Fin de alertas mensuales");

            if (!foundFiles) {
                bot.getApi().sendMessage(query->message->chat->id, "No se encuentran alertas este mes.");
            }

        } else if (callbackData == "button6") {
        // Handle button 6 click
            for (const auto& entry : std::filesystem::directory_iterator(folderFilePath)) {
                if (std::filesystem::is_regular_file(entry) && entry.path().extension() == ".bmp") {
                    const string contenidoFilePath = entry.path().string();
                    string alertaEnv = alertaDate(contenidoFilePath);
                    bot.getApi().sendPhoto(query->message->chat->id, TgBot::InputFile::fromFile(contenidoFilePath, photoMimeType));
                    bot.getApi().sendMessage(query->message->chat->id, "Alerta enviada el: "+alertaEnv);
                } else {
                bot.getApi().sendMessage(query->message->chat->id, "No hay alertas registradas.");
                }
            }
            
             bot.getApi().sendMessage(query->message->chat->id, "Fin de todas las alertas");

        } else {
            // Handle other buttons or unknown callback data
            bot.getApi().sendMessage(query->message->chat->id, "Unknown button clicked!");
        }
    });

    signal(SIGINT, [](int s) {
    printf("SIGINT got\n");
    exit(0);
    });
        
    try {
        printf("Bot username: %s\n", bot.getApi().getMe()->username.c_str());
        TgBot::TgLongPoll longPoll(bot);
        while (true) {
            printf("Long poll started\n");
            longPoll.start();
        }
    } catch (std::exception& e) {
        printf("error: %s\n", e.what());
    }}
}



