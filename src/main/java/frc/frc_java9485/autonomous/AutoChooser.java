package frc.frc_java9485.autonomous;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoChooser {
  private final Path autosDir;
  private final Path deployDirectory;

  private final List<String> autos;
  private final LoggedDashboardChooser<String> chooser;

  public AutoChooser(String chooserName, String defaultOption) {
    if (chooserName == null) {
      throw new IllegalArgumentException("O nome nao pode ser nulo");
    }

    if (defaultOption == null) {
      throw new IllegalArgumentException("A opcao padrao nao pode ser nula");
    }

    deployDirectory = Paths.get(Filesystem.getDeployDirectory().getPath());
    autosDir = deployDirectory.resolve("pathplanner/autos");

    autos = new ArrayList<>();
    chooser = new LoggedDashboardChooser<>(chooserName);
    chooser.addDefaultOption(defaultOption, defaultOption);

    try (Stream<Path> files = Files.list(autosDir)) {
      files
          .filter(f -> f.toString().endsWith(".auto"))
          .map(f -> f.getFileName().toString().replace(".auto", ""))
          .forEach(autos::add);
    } catch (IOException e) {
      e.printStackTrace();
    }

    for (String auto : autos) {
      chooser.addOption(auto, auto);
    }
  }

  public LoggedDashboardChooser<String> getChooser() {
    return chooser;
  }

  public String getSelectedOption() {
    return chooser.get();
  }
}
