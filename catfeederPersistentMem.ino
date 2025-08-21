/*
 * Cat Feeder with Persistent Memory
 * 
 * This sketch controls an automatic cat feeder with a stepper motor, OLED display,
 * rotary encoder for user input, and RTC for timekeeping.
 * 
 * Features:
 * - Multiple programmable feeding times
 * - Deep sleep for power efficiency
 * - Persistent storage of feeding schedule
 * - Menu-driven UI with rotary encoder
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Stepper.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <RTClib.h>
#include <Preferences.h>

// ===== Hardware Configuration =====
// Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Rotary Encoder Pins
#define ROTARY_CLK 0
#define ROTARY_DT 1
#define ROTARY_SW 3

// Stepper Motor Pins
#define STEPPER_POWER 4
#define IN1 7
#define IN2 8
#define IN3 10
#define IN4 20

// RTC Pin
#define RTC_PIN 2

// I2C Pins
#define I2C_SDA 5
#define I2C_SCL 6

// ===== Timing Constants =====
#define DEBOUNCE_DELAY 60          // Milliseconds for rotary encoder debounce
#define BUTTON_DEBOUNCE_DELAY 250 // Milliseconds for button debounce
#define SLEEP_TIMEOUT 30000       // Milliseconds before sleep (30 seconds)
#define SLEEP_MESSAGE_DURATION 5000 // Display sleep message for 5 seconds
#define STEPS_PER_REVOLUTION 2048 // Steps for 28BYJ-48 stepper

// Maximum sleep duration (24 hours)
// ESP32's sleep timer uses uint64_t so we can set much longer durations
#define MAX_SLEEP_DURATION 24ULL * 60 * 60 * 1000000 // 24 hours in microseconds
#define MAX_SLEEP_SECONDS 24ULL * 60 * 60            // 24 hours in seconds

// ===== Feed Time Settings =====
#define MAX_FEED_TIMES 10
#define DEFAULT_HOUR 12
#define DEFAULT_MINUTE 0

// ===== Menu Constants =====
#define MENU_ITEM_CURRENT_TIME 0
#define MENU_ITEM_SET_FEED_TIME 1
#define MENU_ITEM_FEED_NOW 2
#define MENU_ITEM_SET_TIME 3
#define MENU_ITEM_COUNT 4

// ===== Device Objects =====
Stepper myStepper(STEPS_PER_REVOLUTION, IN1, IN3, IN2, IN4);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RTC_DS1307 rtc;
Preferences preferences;

// ===== Time Variables =====
// Stored in RTC memory to persist during sleep
RTC_DATA_ATTR int hours = DEFAULT_HOUR;
RTC_DATA_ATTR int minutes = DEFAULT_MINUTE;
RTC_DATA_ATTR int seconds = 0;
int lastMinute = -1; // Doesn't need to persist during sleep

// ===== Feed Time Storage =====
int feedTimes[MAX_FEED_TIMES][3];  // [hour, minute, rotations]
int feedTimeCount = 0;

// ===== State Variables =====
// Menu state
enum MenuState {
  DISPLAY_TIME,
  MAIN_MENU,
  FEED_TIME_MENU,
  SET_TIME
};

// Store the full sleep duration
RTC_DATA_ATTR unsigned long fullSleepSeconds = 0;

// Current UI state
MenuState currentState = DISPLAY_TIME;
int menuSelection = 0;  // 0: Current Time, 1: Set Feed Time, 2: Feed Now

// Feed time editor state
bool addingNewFeedTime = false;
bool editingHours = true;
bool editingRotations = false;
int newFeedTimeRotations = 1;
int feedTimeSelection = 0;
int newFeedTimeHours = DEFAULT_HOUR;
int newFeedTimeMinutes = DEFAULT_MINUTE;

// Time editor state
int newHours = DEFAULT_HOUR;
int newMinutes = DEFAULT_MINUTE;

// ===== Hardware State =====
volatile int lastCLKState;
volatile bool encoderChanged = false;
volatile bool encoderDirection = false; // true for clockwise, false for counter-clockwise
unsigned long lastButtonPress = 0;
unsigned long lastInteractionTime = 0;
bool motorSpunForCurrentMinute = false;

// ===== Function Prototypes =====
void loadFeedTimes();
void saveFeedTimes();
void updateTimeFromRTC();
void checkFeedTimes();
void dispenseFood(int rotations);
void prepareForSleep();
uint64_t calculateSleepDuration();
void displayCurrentState();
void displaySleepMessage(unsigned long sleepSeconds);
void showFeedTimeDebug();

// ===== Setup Function =====
void setup() {
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for (;;);
  }
  
  // Display initial setup
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Initialize RTC pin
  pinMode(RTC_PIN, OUTPUT);
  digitalWrite(RTC_PIN, HIGH);

  // Initialize rotary encoder pins
  pinMode(ROTARY_CLK, INPUT_PULLUP);
  pinMode(ROTARY_DT, INPUT_PULLUP);
  pinMode(ROTARY_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), handleEncoder, CHANGE);
  lastCLKState = digitalRead(ROTARY_CLK);

  // Initialize stepper motor pins
  pinMode(STEPPER_POWER, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  myStepper.setSpeed(15);

  // Initialize last interaction time
  lastInteractionTime = millis();

  // Initialize RTC
  if (!rtc.begin()) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Couldn't find RTC");
    display.display();
    while (1); // Halt if RTC not available
  }

  if (!rtc.isrunning()) {
    // RTC is not running, but we will proceed anyway.
  }

  // Load saved feed times
  loadFeedTimes();

  // Update time if waking from sleep
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    updateTimeFromRTC();
    checkFeedTimes();
  }
}

// ===== RTC Functions =====
/**
 * Update time variables from RTC after waking from sleep
 */
void updateTimeFromRTC() {
  DateTime now = rtc.now();
  hours = now.hour();
  minutes = now.minute();
  seconds = now.second();
}

/**
 * Update time and handle minute changes
 */
void updateTimeAndCheckMinute() {
  updateTimeFromRTC();

  // Reset motor spin flag at the start of a new minute
  if (minutes != lastMinute) {
    motorSpunForCurrentMinute = false;
    lastMinute = minutes;
  }
}

// ===== Feeding Functions =====
/**
 * Check if any programmed feed times match current time
 */
void checkFeedTimes() {
  // Skip if already fed in this minute
  if (motorSpunForCurrentMinute) {
    return;
  }
  
  // Check each feed time
  for (int i = 0; i < feedTimeCount; i++) {
    if (hours == feedTimes[i][0] && minutes == feedTimes[i][1]) {
      dispenseFood(feedTimes[i][2]);
      motorSpunForCurrentMinute = true;
      return;
    }
  }
}

/**
 * Rotate the stepper motor one revolution to dispense food
 * Includes anti-jam back-and-forth movements during rotation
 */
void dispenseFood(int rotations) {
  // Update interaction time to prevent sleep during feeding
  lastInteractionTime = millis();

  // Enable stepper motor power
  digitalWrite(STEPPER_POWER, HIGH);

  // Show status on display
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Feeding...");
  display.display();

  for (int r = 0; r < rotations; r++) {
    // Rotate backward slightly to clear any jams
    myStepper.step(-STEPS_PER_REVOLUTION / 8); // Rotate 1/8th of a revolution backward
    delay(50); // Short delay

    // Rotate forward for one full revolution with anti-jam movements
    for (int i = 0; i < 4; i++) {
      myStepper.step(STEPS_PER_REVOLUTION / 4);
      delay(20); // Short delay for the motor to settle
      myStepper.step(-STEPS_PER_REVOLUTION / 16); // Small step back
      delay(20);
    }
  }

  // Disable stepper motor power to save energy
  digitalWrite(STEPPER_POWER, LOW);
}

// ===== Encoder Input Handling =====
/**
 * Interrupt handler for rotary encoder
 */
void handleEncoder() {
  int currentCLKState = digitalRead(ROTARY_CLK);
  if (currentCLKState != lastCLKState && currentCLKState == HIGH) {
    encoderDirection = (digitalRead(ROTARY_DT) == currentCLKState);
    encoderChanged = true;
    lastInteractionTime = millis();
  }
  lastCLKState = currentCLKState;
}

void handleEncoderChange() {
  if (encoderChanged) {
    processEncoderRotation(encoderDirection);
    encoderChanged = false;
  }
}

/**
 * Process encoder rotation based on current UI state
 */
void processEncoderRotation(bool clockwise) {
  // Main menu navigation
  if (currentState == MAIN_MENU) {
    if (clockwise) {
      menuSelection = (menuSelection + 1) % MENU_ITEM_COUNT;
    } else {
      menuSelection = (menuSelection - 1 + MENU_ITEM_COUNT) % MENU_ITEM_COUNT;
    }
    return;
  }
  
  // Feed time menu navigation
  if (currentState == FEED_TIME_MENU) {
    if (addingNewFeedTime) {
      // Editing a new feed time
      if (editingHours) {
        newFeedTimeHours = clockwise ? 
                          (newFeedTimeHours + 1) % 24 : 
                          (newFeedTimeHours - 1 + 24) % 24;
      } else if (editingRotations) {
        newFeedTimeRotations = clockwise ? 
                               (newFeedTimeRotations % 3) + 1 : 
                               (newFeedTimeRotations == 1 ? 3 : newFeedTimeRotations - 1);
      } else {
        newFeedTimeMinutes = clockwise ? 
                            (newFeedTimeMinutes + 1) % 60 : 
                            (newFeedTimeMinutes - 1 + 60) % 60;
      }
    } else {
      // Navigating feed time list
      int totalOptions = feedTimeCount + 2; // Feed times + Add New + Exit
      feedTimeSelection = clockwise ? 
                         (feedTimeSelection + 1) % totalOptions : 
                         (feedTimeSelection - 1 + totalOptions) % totalOptions;
    }
  }

  // Set time menu navigation
  if (currentState == SET_TIME) {
    if (editingHours) {
      newHours = clockwise ? (newHours + 1) % 24 : (newHours - 1 + 24) % 24;
    } else {
      newMinutes = clockwise ? (newMinutes + 1) % 60 : (newMinutes - 1 + 60) % 60;
    }
  }
}

/**
 * Check for button press and handle UI navigation
 */
void checkRotaryButton() {
  if (digitalRead(ROTARY_SW) == LOW) {
    // Debounce button press
    if (millis() - lastButtonPress > BUTTON_DEBOUNCE_DELAY) {
      lastInteractionTime = millis();  // Update interaction time
      
      // Process button press based on current state
      processButtonPress();
      
      lastButtonPress = millis();
    }
  }
}

/**
 * Process button press based on current UI state
 */
void processButtonPress() {
  switch (currentState) {
    case DISPLAY_TIME:
      // Time display -> Main menu
      currentState = MAIN_MENU;
      break;
      
    case MAIN_MENU:
      handleMainMenuSelection();
      break;
      
    case FEED_TIME_MENU:
      handleFeedTimeMenuSelection();
      break;
    case SET_TIME:
      handleSetTimeMenuSelection();
      break;
  }
}

/**
 * Handle selection in the main menu
 */
void handleMainMenuSelection() {
  switch (menuSelection) {
    case MENU_ITEM_CURRENT_TIME: // Current Time
      currentState = DISPLAY_TIME;
      break;
      
    case MENU_ITEM_SET_FEED_TIME: // Set Feed Time
      currentState = FEED_TIME_MENU;
      feedTimeSelection = 0;
      addingNewFeedTime = false;
      editingRotations = false;
      newFeedTimeRotations = 1;
      
      // Debug display to show loaded feed times
      // showFeedTimeDebug();
      break;
      
    case MENU_ITEM_FEED_NOW: // Feed Now
      dispenseFood(1);
      currentState = DISPLAY_TIME;
      break;

    case MENU_ITEM_SET_TIME: // Set Time
      currentState = SET_TIME;
      editingHours = true;
      newHours = hours;
      newMinutes = minutes;
      menuSelection = 0; // Reset menu selection
      break;
  }
}

/**
 * Display debug information about stored feed times
 */
void showFeedTimeDebug() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Feed Time Debug:");
  
  display.print("Count: ");
  display.println(feedTimeCount);
  
  // Show current time
  display.print("Current: ");
  display.print(hours);
  display.print(":");
  if (minutes < 10) display.print("0");
  display.println(minutes);
  
  // Show each stored feed time
  for (int i = 0; i < feedTimeCount && i < 5; i++) {
    display.print(i);
    display.print(": ");
    display.print(feedTimes[i][0]);
    display.print(":");
    if (feedTimes[i][1] < 10) display.print("0");
    display.println(feedTimes[i][1]);
  }

  display.display();
  delay(3000); // Show debug for 3 seconds
}

/**
 * Handle selection in the feed time menu
 */
void handleFeedTimeMenuSelection() {
  if (addingNewFeedTime) {
    // Handle new feed time editing
    if (editingHours) {
      // Switch to editing minutes
      editingHours = false;
    } else if (!editingRotations) { // This means we are editing minutes
      // Switch to editing rotations
      editingRotations = true;
    } else { // This means we are editing rotations
      // Save new feed time
      if (feedTimeCount < MAX_FEED_TIMES) {
        feedTimes[feedTimeCount][0] = newFeedTimeHours;
        feedTimes[feedTimeCount][1] = newFeedTimeMinutes;
        feedTimes[feedTimeCount][2] = newFeedTimeRotations;
        feedTimeCount++;
        saveFeedTimes();
      }
      addingNewFeedTime = false;
    }
  } else {
    // Handle feed time list selection
    if (feedTimeSelection == feedTimeCount) {
      // Add new feed time
      addingNewFeedTime = true;
      editingHours = true;
      // No need to reinitialize these values here as they're already set to defaults
    } else if (feedTimeSelection == feedTimeCount + 1) {
      // Exit to main menu
      currentState = MAIN_MENU;
    } else {
      // Remove selected feed time
      removeFeedTime(feedTimeSelection);
    }
  }
}

/**
 * Handle selection in the set time menu
 */
void handleSetTimeMenuSelection() {
  if (editingHours) {
    // Switch to editing minutes
    editingHours = false;
  } else {
    // Save new time
    hours = newHours;
    minutes = newMinutes;
    rtc.adjust(DateTime(2025, 8, 19, hours, minutes, 0));
    currentState = DISPLAY_TIME;
  }
}

/**
 * Remove a feed time from the list
 */
void removeFeedTime(int index) {
  // Shift all subsequent feed times down one position
  for (int i = index; i < feedTimeCount - 1; i++) {
    feedTimes[i][0] = feedTimes[i + 1][0];
    feedTimes[i][1] = feedTimes[i + 1][1];
  }
  feedTimeCount--;
  saveFeedTimes();
}

// ===== Display Functions =====
/**
 * Display current time on the OLED
 */
void displayTime() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(20, 30);

  // Format hours with leading zero if needed
  if (hours < 10) display.print("0");
  display.print(hours);
  display.print(":");

  // Format minutes with leading zero if needed
  if (minutes < 10) display.print("0");
  display.print(minutes);
  display.print(":");

  // Format seconds with leading zero if needed
  if (seconds < 10) display.print("0");
  display.print(seconds);

  display.display();
}

/**
 * Display the main menu
 */
void displayMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.println("Select Option:");
  display.println(menuSelection == MENU_ITEM_CURRENT_TIME ? "> Current Time" : "  Current Time");
  display.println(menuSelection == MENU_ITEM_SET_FEED_TIME ? "> Set Feed Time" : "  Set Feed Time");
  display.println(menuSelection == MENU_ITEM_FEED_NOW ? "> Feed Now" : "  Feed Now");
  display.println(menuSelection == MENU_ITEM_SET_TIME ? "> Set Time" : "  Set Time");

  display.display();
}

/**
 * Display the feed time menu
 */
void displayFeedTimeMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  if (addingNewFeedTime) {
    displayNewFeedTimeEditor();
  } else {
    displayFeedTimeList();
  }

  display.display();
}

/**
 * Display the feed time editor
 */
void displayNewFeedTimeEditor() {
  display.println("New Feed Time:");
  display.setTextSize(2);
  
  // Show indicator for what's being edited
  display.print(editingHours ? "> " : (editingRotations ? "> " : "  "));
  
  // Display hours
  display.print(newFeedTimeHours);
  display.print(":");
  
  // Display minutes with leading zero
  if (newFeedTimeMinutes < 10) display.print("0");
  display.println(newFeedTimeMinutes);
  
  // Display rotations
  display.print(editingRotations ? "> (" : "  (");
  display.print(newFeedTimeRotations);
  display.println(" rot)");
  
  // Show instruction
  display.setTextSize(1);
  if (editingHours) {
    display.println("Edit Minutes");
  } else if (editingRotations) {
    display.println("Save");
  } else {
    display.println("Edit Rotations");
  }
}

/**
 * Display the list of feed times
 */
void displayFeedTimeList() {
  display.setTextSize(1);
  display.println("Feed Times:");
  
  // Display each feed time
  for (int i = 0; i < feedTimeCount; i++) {
    display.print(i == feedTimeSelection ? "> " : "  ");
    display.print(feedTimes[i][0]);
    display.print(":");
    if (feedTimes[i][1] < 10) display.print("0");
    display.print(feedTimes[i][1]);
    display.print(" (");
    display.print(feedTimes[i][2]);
    display.println(" rot)");
  }
  
  // Display menu options
  display.println(feedTimeSelection == feedTimeCount ? "> Add New Feed Time" : "  Add New Feed Time");
  display.println(feedTimeSelection == feedTimeCount + 1 ? "> Exit" : "  Exit");
}

/**
 * Display the set time menu
 */
void displaySetTimeMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Set Time:");
  display.setTextSize(2);
  
  // Show indicator for what's being edited
  display.print(editingHours ? "> " : "  ");
  
  // Display hours
  display.print(newHours);
  display.print(":");
  
  // Display minutes with leading zero
  if (newMinutes < 10) display.print("0");
  display.println(newMinutes);
  
  // Show instruction
  display.setTextSize(1);
  display.println(editingHours ? "Edit Minutes" : "Save");
  display.display();
}

/**
 * Display the current UI state
 */
void displayCurrentState() {
  switch (currentState) {
    case DISPLAY_TIME:
      displayTime();
      break;
    case MAIN_MENU:
      displayMenu();
      break;
    case FEED_TIME_MENU:
      displayFeedTimeMenu();
      break;
    case SET_TIME:
      displaySetTimeMenu();
      break;
  }
}

// ===== Sleep Management =====
/**
 * Display sleep message with essential information for small screens
 */
void displaySleepMessage(unsigned long sleepSeconds) {
  // Calculate actual sleep duration (limited by hardware)
  unsigned long actualSleepSeconds = sleepSeconds;
  if (actualSleepSeconds * 1000000ULL > MAX_SLEEP_DURATION) {
    actualSleepSeconds = MAX_SLEEP_DURATION / 1000000ULL;
  }
  
  // Clear display and prepare for text
  display.clearDisplay();
  display.setTextSize(1);
  
  // Show current time
  display.setCursor(0, 0);
  display.print("Now: ");
  display.print(hours);
  display.print(":");
  if (minutes < 10) display.print("0");
  display.print(minutes);
  

  if (feedTimeCount > 0) {
    // Show sleep duration (safely calculate hours and minutes)
    display.setCursor(0, 10);
    display.print("To feed: ");
    
    // Safe calculation of hours and minutes from seconds
    unsigned long fullHours = sleepSeconds / 3600UL;
    unsigned long fullMins = (sleepSeconds % 3600UL) / 60UL;
    
    // Cap display values to reasonable limits just in case
    if (fullHours > 99) fullHours = 99;
    
      display.print(fullHours);
      display.print("h ");
      display.print(fullMins);
      display.println("m");
  }
  
  // If we're limiting sleep time due to hardware constraints, show actual wake time
  if (sleepSeconds != actualSleepSeconds || feedTimeCount == 0) {
    display.setCursor(0, 15);
    display.print("Next wake: ");
    
    unsigned long wakeHour = hours;
    unsigned long wakeMinute = minutes;
    
    // Calculate minutes from actualSleepSeconds
    unsigned long actualSleepMinutes = actualSleepSeconds / 60UL;
    
    // Add minutes
    wakeMinute += actualSleepMinutes;
    // Handle minute overflow
    if (wakeMinute >= 60) {
      wakeHour += wakeMinute / 60;
      wakeMinute %= 60;
    }
    
    // Handle hour overflow
    if (wakeHour >= 24) {
      wakeHour %= 24;
    }
    
    display.print((int)wakeHour);
    display.print(":");
    if (wakeMinute < 10) display.print("0");
    display.println((int)wakeMinute);
  }
  
  // Show next feed times (up to 2 if we're using limited sleep)
  int yPos = sleepSeconds != actualSleepSeconds ? 35 : 25;
  display.setCursor(0, yPos);
  display.print("Feed times:");
  
  if (feedTimeCount == 0) {
    display.setCursor(0, yPos + 10);
    display.print("None set");
  } else {
    // Sort feed times (simple bubble sort)
    int sortedIndices[MAX_FEED_TIMES];
    for (int i = 0; i < feedTimeCount; i++) {
      sortedIndices[i] = i;
    }
    
    for (int i = 0; i < feedTimeCount-1; i++) {
      for (int j = 0; j < feedTimeCount-i-1; j++) {
        // Convert to minutes for easier comparison
        int time1 = feedTimes[sortedIndices[j]][0] * 60 + feedTimes[sortedIndices[j]][1];
        int time2 = feedTimes[sortedIndices[j+1]][0] * 60 + feedTimes[sortedIndices[j+1]][1];
        if (time1 > time2) {
          // Swap
          int temp = sortedIndices[j];
          sortedIndices[j] = sortedIndices[j+1];
          sortedIndices[j+1] = temp;
        }
      }
    }
    
    // Display feed times (limit based on screen space)
    int maxDisplay = sleepSeconds != actualSleepSeconds ? 2 : 3;
    int displayCount = min(maxDisplay, feedTimeCount);
    for (int i = 0; i < displayCount; i++) {
      display.setCursor(0, yPos + 10 + i * 10);
      int idx = sortedIndices[i];
      display.print(feedTimes[idx][0]);
      display.print(":");
      if (feedTimes[idx][1] < 10) display.print("0");
      display.print(feedTimes[idx][1]);
    }
  }
  
  display.display();
}

/**
 * Prepare device for deep sleep
 */
void prepareForSleep() {
  // Calculate sleep duration
  uint64_t sleepDuration = calculateSleepDuration();
  unsigned long sleepDurationSeconds = sleepDuration / 1000000ULL;
  
  // Safety check - if calculated sleep is less than 60 seconds, use 5 minutes
  // This prevents immediate wake-up cycles
  if (sleepDurationSeconds < 60) {
    sleepDurationSeconds = 300; // 5 minutes
    sleepDuration = sleepDurationSeconds * 1000000ULL;
  }
  
  // Display sleep message - show full sleep time until next feed, even if we're
  // sleeping for less due to microsecond limitations
  displaySleepMessage(fullSleepSeconds);
  delay(SLEEP_MESSAGE_DURATION);

  // Clear and turn off display
  display.clearDisplay();
  display.display();
  display.ssd1306_command(SSD1306_DISPLAYOFF);

  // Power down unnecessary hardware
  Wire.end();
  pinMode(STEPPER_POWER, INPUT_PULLUP);
  pinMode(IN1, INPUT_PULLUP);
  pinMode(IN2, INPUT_PULLUP);
  pinMode(IN3, INPUT_PULLUP);
  pinMode(IN4, INPUT_PULLUP);
  pinMode(RTC_PIN, INPUT_PULLUP);
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);

  // Configure GPIO wakeup sources
  uint64_t gpioWakeupMask = 0;
  gpioWakeupMask |= (1ULL << ROTARY_DT);
  gpioWakeupMask |= (1ULL << ROTARY_SW);
  
  // Configure wakeup with LOW level (when encoder is turned/pressed)
  esp_sleep_enable_gpio_wakeup();
  esp_deep_sleep_enable_gpio_wakeup(gpioWakeupMask, ESP_GPIO_WAKEUP_GPIO_LOW);

  // Enable timer wakeup and enter deep sleep
  esp_sleep_enable_timer_wakeup(sleepDuration);
  esp_deep_sleep_start();
}

/**
 * Calculate sleep duration until next feed time
 * Returns sleep duration in microseconds
 */
uint64_t calculateSleepDuration() {
  // No feed times configured - sleep for 1 hour
  if (feedTimeCount == 0) {
    fullSleepSeconds = 24 * 60 * 60; // 24 hours
    return 24ULL * 60 * 60 * 1000000; // 24 hours in microseconds
  }
  
  // Get current time in minutes since midnight
  int currentTimeInMinutes = (hours * 60) + minutes;
  
  // Find the earliest upcoming feed time
  int earliestFeedTimeInMinutes = 24 * 60; // Initialize to max (24 hours in minutes)
  bool foundFeedTimeToday = false;
  
  // First look for feed times later today
  for (int i = 0; i < feedTimeCount; i++) {
    int feedTimeInMinutes = (feedTimes[i][0] * 60) + feedTimes[i][1];
    
    // Only consider feed times that are in the future
    if (feedTimeInMinutes > currentTimeInMinutes) {
      if (feedTimeInMinutes < earliestFeedTimeInMinutes) {
        earliestFeedTimeInMinutes = feedTimeInMinutes;
        foundFeedTimeToday = true;
      }
    }
  }
  
  // If no feed time found later today, find earliest feed time tomorrow
  if (!foundFeedTimeToday) {
    earliestFeedTimeInMinutes = 24 * 60; // Reset to max
    
    for (int i = 0; i < feedTimeCount; i++) {
      int feedTimeInMinutes = (feedTimes[i][0] * 60) + feedTimes[i][1];
      if (feedTimeInMinutes < earliestFeedTimeInMinutes) {
        earliestFeedTimeInMinutes = feedTimeInMinutes;
      }
    }
    
    // Add 24 hours since it's tomorrow
    earliestFeedTimeInMinutes += 24 * 60;
  }
  
  // Calculate sleep duration in minutes (ensure it's never negative)
  int sleepMinutes = earliestFeedTimeInMinutes - currentTimeInMinutes;
  if (sleepMinutes <= 0) {
    sleepMinutes = 60; // Default to 1 hour
  }
  
  // Convert to seconds
  unsigned long sleepSeconds = (unsigned long)sleepMinutes * 60UL;
  
  // Store the full sleep duration for display purposes
  fullSleepSeconds = sleepSeconds;
  
  // Limit sleep duration to MAX_SLEEP_SECONDS (24 hours)
  if (sleepSeconds > MAX_SLEEP_SECONDS) {
    sleepSeconds = MAX_SLEEP_SECONDS;
  }
  
  // Convert to microseconds and return
  return (uint64_t)sleepSeconds * 1000000ULL;
}

// ===== Persistent Storage Functions =====
/**
 * Save feed times to persistent storage
 */
void saveFeedTimes() {
  preferences.begin("feeder", false); // Open namespace in RW mode
  
  // Save feed time count
  preferences.putInt("count", feedTimeCount);
  
  // Save each feed time as a combined value (hour*100 + minute)
  for (int i = 0; i < feedTimeCount; i++) {
    char key[8];
    snprintf(key, sizeof(key), "feed_%d", i);
    int timeValue = feedTimes[i][0] * 10000 + feedTimes[i][1] * 100 + feedTimes[i][2]; // Store as HHMMR
    preferences.putInt(key, timeValue);
  }
  
  preferences.end();
}

/**
 * Load feed times from persistent storage
 */
void loadFeedTimes() {
  preferences.begin("feeder", true); // Open namespace in read-only mode
  
  // Load feed time count
  feedTimeCount = preferences.getInt("count", 0);
  
  // Load each feed time
  for (int i = 0; i < feedTimeCount; i++) {
    char key[8];
    snprintf(key, sizeof(key), "feed_%d", i);
    int timeValue = preferences.getInt(key, 0);
    
    // Extract hours and minutes
    feedTimes[i][0] = timeValue / 10000; // Hours
    feedTimes[i][1] = (timeValue / 100) % 100; // Minutes
    feedTimes[i][2] = timeValue % 100; // Rotations
  }
  
  preferences.end();
}

// ===== Main Loop =====
void loop() {
  // Update time from RTC
  updateTimeAndCheckMinute();
  
  // Check for user input
  handleEncoderChange();
  checkRotaryButton();
  
  // Display current UI state
  displayCurrentState();
  
  // Check for sleep timeout
  if (millis() - lastInteractionTime >= SLEEP_TIMEOUT) {
    prepareForSleep();
  }
  
  // No delay here, let the CPU sleep when idle
}