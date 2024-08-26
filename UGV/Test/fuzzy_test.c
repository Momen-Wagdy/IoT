#include <stdio.h>

// Min function
float min(float a1, float a2) {
    return a1 > a2 ? a2 : a1;
}

// Max function
float max(float a1, float a2) {
    return a1 < a2 ? a2 : a1;
}

// Defuzzification using centroid method
float defuzzification(float low, float med, float high) {
    float low_a = 111 - 1e-10;
    float low_b = 111;
    float low_c = 155;
    float med_a = 140;
    float med_b = 167.5;
    float med_c = 195;
    float high_a = 180;
    float high_b = 255;
    float high_c = 255 + 1e-10;

    float low1 = low * (low_b - low_a) + low_a;
    float low2 = low_c - low * (low_c - low_b);

    float med1 = med * (med_b - med_a) + med_a;
    float med2 = med_c - med * (med_c - med_b);

    float high1 = high * (high_b - high_a) + high_a;
    float high2 = high_c - high * (high_c - high_b);

    float sum = low + med + high;
    if (sum == 0) {
        return 0; // Avoid division by zero
    }

    return (low1 * low + low2 * low + med1 * med + med2 * med + high1 * high + high2 * high) / (2 * sum);
}

// Triangular membership function
float triangular_MF(float x, float a, float b, float c) {
    float first_term = (x - a) / (b - a);
    float second_term = (c - x) / (c - b);
    float inner_term = first_term > second_term ? second_term : first_term;
    float final_membership = inner_term > 0 ? inner_term : 0;
    return final_membership;
}

// Asymmetric triangular membership function
float triangular_MFa(float x, float a, float c) {
    float b = (a + c) / 2;
    float first_term = (x - a) / (b - a);
    float second_term = (c - x) / (c - b);
    float inner_term = first_term > second_term ? second_term : first_term;
    float final_membership = inner_term > 0 ? inner_term : 0;
    return final_membership;
}

// Fuzzy Inference System
void FIS(float orientation, float distance) {
    // Fuzzification of orientation and distance
    float right = triangular_MF(orientation, -320 - 1e-10, -320, -50);
    float center = triangular_MFa(orientation, -75, 75);
    float left = triangular_MF(orientation, 50, 320, 320 + 1e-10);
    float close = triangular_MF(distance, -1e-10, 0, 25);
    float medium = triangular_MFa(distance, 20, 55);
    float far = triangular_MF(distance, 50, 90, 90 + 1e-10);

    // Print fuzzified values
    printf("Fuzzified Values:\n");
    printf("Right: %f, Center: %f, Left: %f\n", right, center, left);
    printf("Close: %f, Medium: %f, Far: %f\n", close, medium, far);

    // Initialize output possibilities
    float speed_left_very_low = 0;
    float speed_left_low = 0;
    float speed_left_medium = 0;
    float speed_left_high = 0;
    float speed_right_very_low = 0;
    float speed_right_low = 0;
    float speed_right_medium = 0;
    float speed_right_high = 0;
    float direction_left = 0;
    float direction_center = 0;
    float direction_right = 0;
  
    // Apply fuzzy rules
    float rule1 = min(right, far);
    speed_left_medium = max(speed_left_medium, rule1);
    speed_right_high = max(speed_right_high, rule1);
    direction_right = max(direction_right, rule1);

    float rule2 = min(left, far);
    speed_left_high = max(speed_left_high, rule2);
    speed_right_medium = max(speed_right_medium, rule2);
    direction_left = max(direction_left, rule2);

    float rule3 = min(right, medium);
    speed_left_low = max(speed_left_low, rule3);
    speed_right_medium = max(speed_right_medium, rule3);
    direction_right = max(direction_right, rule3);

    float rule4 = min(left, medium);
    speed_left_medium = max(speed_left_medium, rule4);
    speed_right_low = max(speed_right_low, rule4);
    direction_left = max(direction_left, rule4);

    float rule5 = min(right, close);
    speed_left_very_low = max(speed_left_very_low, rule5);
    speed_right_low = max(speed_right_low, rule5);
    direction_right = max(direction_right, rule5);

    float rule6 = min(left, close);
    speed_left_low = max(speed_left_low, rule6);
    speed_right_very_low = max(speed_right_very_low, rule6);
    direction_left = max(direction_left, rule6);

    float rule7 = min(center, far);
    speed_left_high = max(speed_left_high, rule7);
    speed_right_high = max(speed_right_high, rule7);
    direction_center = max(direction_center, rule7);

    float rule8 = min(center, medium);
    speed_left_medium = max(speed_left_medium, rule8);
    speed_right_medium = max(speed_right_medium, rule8);
    direction_center = max(direction_center, rule8);

    float rule9 = min(center, close);
    speed_left_low = max(speed_left_low, rule9);
    speed_right_low = max(speed_right_low, rule9);
    direction_center = max(direction_center, rule9);

    // Print rule activations
    printf("\nRule Activations:\n");
    printf("Rule1: %f, Rule2: %f, Rule3: %f\n", rule1, rule2, rule3);
    printf("Rule4: %f, Rule5: %f, Rule6: %f\n", rule4, rule5, rule6);
    printf("Rule7: %f, Rule8: %f, Rule9: %f\n", rule7, rule8, rule9);

    // Defuzzification to get the crisp output
    float speed_left = defuzzification(speed_left_low, speed_left_medium, speed_left_high);
    float speed_right = defuzzification(speed_right_low, speed_right_medium, speed_right_high);
    float direction = defuzzification(direction_left, direction_center, direction_right);

    // Print final defuzzified output
    printf("\nDefuzzified Output:\n");
    printf("Speed Left: %f, Speed Right: %f, Direction: %f\n", speed_left, speed_right, direction);
}

int main() {
    // Example test case
    FIS(300, 5);
    return 0;
}
