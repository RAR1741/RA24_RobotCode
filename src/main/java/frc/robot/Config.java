package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.tomlj.Toml;
import org.tomlj.TomlParseError;
import org.tomlj.TomlParseResult;

import edu.wpi.first.wpilibj.Filesystem;

public class Config {
  private static TomlParseResult m_parsedConfig;

  /**
   * Load the configs from the specified file
   *
   * @param filepath Path to the config file
   */
  public static void loadFromFile(String fileName) {
    try {
      Path source = Paths.get(new File(Filesystem.getDeployDirectory(), fileName).getPath());
      m_parsedConfig = Toml.parse(source);
    } catch (IOException e) {
      System.out.println("Failed to load configuration: " + e.getMessage());
      m_parsedConfig = null; // Gross but a quick way to implement this interface
      // TODO Change this interface
      return;
    }

    if (!m_parsedConfig.errors().isEmpty()) {
      System.err.printf("Error(s) parsing config file '%s':\n", fileName);

      for (TomlParseError error : m_parsedConfig.errors()) {
        System.err.println(error.getMessage());
      }

      m_parsedConfig = null;

      return;
    }

    System.out.println("Config succesfully loaded...");
  }

  /**
   * Get data assigned to the specified key
   *
   * @param key          The key
   * @param defaultValue Value to be returned if no valid data is found
   *
   * @return The data found at the specified key
   */
  public static double getData(String key, double defaultValue) {
    if (m_parsedConfig == null) {
      return defaultValue;
    }

    return m_parsedConfig.getDouble(
        key, new DoubleSupplier() {
          @Override
          public double getAsDouble() {
            return defaultValue;
          }
        });
  }

  /**
   * Get data assigned to the specified key
   *
   * @param key          The key
   * @param defaultValue Value to be returned if no valid data is found
   *
   * @return The data found at the specified key
   */
  public static boolean getData(String key, boolean defaultValue) {
    if (m_parsedConfig == null) {
      return defaultValue;
    }

    return m_parsedConfig.getBoolean(
        key, new BooleanSupplier() {
          @Override
          public boolean getAsBoolean() {
            return defaultValue;
          }
        });
  }
}
