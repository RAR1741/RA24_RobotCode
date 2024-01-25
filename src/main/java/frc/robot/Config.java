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

public class Config {
    static TomlParseResult m_parsedConfig;

    public static void loadFromFile(String filepath) {
        try {
            Path file = Paths.get(new File(filepath).getName());

            m_parsedConfig = Toml.parse(file);
        } catch (IOException exception) {
            System.err.printf("Unable to read from config file '%s':\n%s\n", filepath, exception.getMessage());

            m_parsedConfig = null;

            return;
        }

        if (!m_parsedConfig.errors().isEmpty()) {
            System.err.printf("Error(s) parsing config file '%s':\n", filepath);

            for (TomlParseError error : m_parsedConfig.errors()) {
                System.err.println(error.getMessage());
            }

            m_parsedConfig = null;

            return;
        }

        System.out.println("Config succesfully loaded...");
    }

    public static double getDouble(String key, double defaultValue) {
        if (m_parsedConfig == null) {
            return defaultValue;
        }

        return m_parsedConfig.getDouble(
            key, new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    return defaultValue;
                }
            }
        );
    }

    public static boolean getBoolean(String key, boolean defaultValue) {
        if (m_parsedConfig == null) {
            return defaultValue;
        }

        return m_parsedConfig.getBoolean(
            key, new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return defaultValue;
                }
            }
        );
    }
}