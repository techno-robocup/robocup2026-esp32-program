find . -name "*.cpp" -o -name "*.h" -o -name "*.c" -o -name "*.hpp -o -name "*.ini" | while read file; do
    rsync -av --mkpath --delete --exclude .direnv/ --exclude .git/ "$file" robo@roboberry.local:robocup2026-esp32-kanto/$file
done
