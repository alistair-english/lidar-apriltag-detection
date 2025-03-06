#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <string>

void printUsage(const char *programName) {
    std::cout << "Usage: " << programName << " <dictionary> <id> <size> <output_file> [border_bits]" << std::endl;
    std::cout << "  dictionary: Dictionary name (DICT_4X4_50, DICT_4X4_100, DICT_4X4_250, DICT_4X4_1000," << std::endl;
    std::cout << "               DICT_5X5_50, DICT_5X5_100, DICT_5X5_250, DICT_5X5_1000," << std::endl;
    std::cout << "               DICT_6X6_50, DICT_6X6_100, DICT_6X6_250, DICT_6X6_1000," << std::endl;
    std::cout << "               DICT_7X7_50, DICT_7X7_100, DICT_7X7_250, DICT_7X7_1000," << std::endl;
    std::cout << "               DICT_ARUCO_ORIGINAL, DICT_APRILTAG_16h5, DICT_APRILTAG_25h9," << std::endl;
    std::cout << "               DICT_APRILTAG_36h10, DICT_APRILTAG_36h11)" << std::endl;
    std::cout << "  id: Marker ID in the dictionary" << std::endl;
    std::cout << "  size: Output marker image size in pixels" << std::endl;
    std::cout << "  output_file: Path to save the marker image (PNG format)" << std::endl;
    std::cout << "  border_bits: Optional border bits around marker (default: 1)" << std::endl;
    std::cout << std::endl;
    std::cout << "Example: " << programName << " DICT_6X6_250 24 200 marker24.png 2" << std::endl;
}

cv::aruco::PREDEFINED_DICTIONARY_NAME getDictionaryByName(const std::string &name) {
    if (name == "DICT_4X4_50")
        return cv::aruco::DICT_4X4_50;
    if (name == "DICT_4X4_100")
        return cv::aruco::DICT_4X4_100;
    if (name == "DICT_4X4_250")
        return cv::aruco::DICT_4X4_250;
    if (name == "DICT_4X4_1000")
        return cv::aruco::DICT_4X4_1000;
    if (name == "DICT_5X5_50")
        return cv::aruco::DICT_5X5_50;
    if (name == "DICT_5X5_100")
        return cv::aruco::DICT_5X5_100;
    if (name == "DICT_5X5_250")
        return cv::aruco::DICT_5X5_250;
    if (name == "DICT_5X5_1000")
        return cv::aruco::DICT_5X5_1000;
    if (name == "DICT_6X6_50")
        return cv::aruco::DICT_6X6_50;
    if (name == "DICT_6X6_100")
        return cv::aruco::DICT_6X6_100;
    if (name == "DICT_6X6_250")
        return cv::aruco::DICT_6X6_250;
    if (name == "DICT_6X6_1000")
        return cv::aruco::DICT_6X6_1000;
    if (name == "DICT_7X7_50")
        return cv::aruco::DICT_7X7_50;
    if (name == "DICT_7X7_100")
        return cv::aruco::DICT_7X7_100;
    if (name == "DICT_7X7_250")
        return cv::aruco::DICT_7X7_250;
    if (name == "DICT_7X7_1000")
        return cv::aruco::DICT_7X7_1000;
    if (name == "DICT_ARUCO_ORIGINAL")
        return cv::aruco::DICT_ARUCO_ORIGINAL;
    if (name == "DICT_APRILTAG_16h5")
        return cv::aruco::DICT_APRILTAG_16h5;
    if (name == "DICT_APRILTAG_25h9")
        return cv::aruco::DICT_APRILTAG_25h9;
    if (name == "DICT_APRILTAG_36h10")
        return cv::aruco::DICT_APRILTAG_36h10;
    if (name == "DICT_APRILTAG_36h11")
        return cv::aruco::DICT_APRILTAG_36h11;

    throw std::invalid_argument("Unknown dictionary name: " + name);
}

int main(int argc, char **argv) {
    // Check command line arguments
    if (argc < 5 || argc > 6) {
        printUsage(argv[0]);
        return 1;
    }

    try {
        // Parse command line arguments
        std::string dictionaryName = argv[1];
        int markerId = std::stoi(argv[2]);
        int markerSize = std::stoi(argv[3]);
        std::string outputFile = argv[4];
        int borderBits = (argc == 6) ? std::stoi(argv[5]) : 1;

        // Validate inputs
        if (markerSize <= 0) {
            std::cerr << "Error: Marker size must be positive" << std::endl;
            return 1;
        }

        if (borderBits < 0) {
            std::cerr << "Error: Border bits must be non-negative" << std::endl;
            return 1;
        }

        // Get the dictionary
        cv::aruco::PREDEFINED_DICTIONARY_NAME dict = getDictionaryByName(dictionaryName);
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dict);

        // Check if marker ID is valid for the selected dictionary
        if (markerId < 0 || markerId >= dictionary->bytesList.rows) {
            std::cerr << "Error: Marker ID " << markerId << " is out of range for dictionary " << dictionaryName
                      << " (max ID: " << dictionary->bytesList.rows - 1 << ")" << std::endl;
            return 1;
        }

        // Create the marker image
        cv::Mat markerImage;
        cv::aruco::drawMarker(dictionary, markerId, markerSize, markerImage, borderBits);

        // Save the marker image
        bool success = cv::imwrite(outputFile, markerImage);
        if (!success) {
            std::cerr << "Error: Failed to save marker image to " << outputFile << std::endl;
            return 1;
        }

        std::cout << "Successfully created marker ID " << markerId << " from dictionary " << dictionaryName
                  << " and saved to " << outputFile << std::endl;
        std::cout << "Marker size: " << markerSize << "x" << markerSize << " pixels, border bits: " << borderBits
                  << std::endl;

        return 0;
    } catch (const std::invalid_argument &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        printUsage(argv[0]);
        return 1;
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
