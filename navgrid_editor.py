import pygame
import json
import os
import sys
from typing import List, Dict, Any, Tuple

# Initialize pygame
pygame.init()

# Constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
BG_COLOR = (50, 50, 50)
GRID_COLOR = (70, 70, 70)
GRID_SIZE = 1  # Grid lines every 1 meter
ACTIVE_COLOR = (0, 255, 0)
COMPLETE_COLOR = (0, 200, 255)
POINT_RADIUS = 5
LINE_WIDTH = 2
FONT_SIZE = 16

# Field dimensions in meters
FIELD_LENGTH = 16.54  # meters
FIELD_WIDTH = 8.02    # meters

class NavGridEditor:
    def __init__(self):
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Navigation Grid Editor")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Arial', FONT_SIZE)
        
        # Calculate scaling factors for meters to pixels
        self.scale_x = SCREEN_WIDTH / FIELD_LENGTH
        self.scale_y = SCREEN_HEIGHT / FIELD_WIDTH
        
        # Use the smaller scale to maintain aspect ratio
        self.scale = min(self.scale_x, self.scale_y)
        
        # Calculate centered position of field
        self.field_pixel_width = FIELD_LENGTH * self.scale
        self.field_pixel_height = FIELD_WIDTH * self.scale
        self.field_offset_x = (SCREEN_WIDTH - self.field_pixel_width) / 2
        self.field_offset_y = (SCREEN_HEIGHT - self.field_pixel_height) / 2
        
        self.polygons = []
        self.current_polygon = []
        self.is_drawing = False
        self.filename = "navgrid.json"
        self.bg_image = None
        self.bg_image_path = ""
        
        # Try to load existing navgrid.json if it exists
        self.load_navgrid()
    
    def meters_to_pixels(self, x_meters, y_meters):
        """Convert coordinates from meters to pixels"""
        x_pixels = self.field_offset_x + (x_meters * self.scale)
        y_pixels = self.field_offset_y + ((FIELD_WIDTH - y_meters) * self.scale)  # Y is inverted in pygame
        return (x_pixels, y_pixels)
    
    def pixels_to_meters(self, x_pixels, y_pixels):
        """Convert coordinates from pixels to meters"""
        x_meters = (x_pixels - self.field_offset_x) / self.scale
        y_meters = FIELD_WIDTH - ((y_pixels - self.field_offset_y) / self.scale)  # Y is inverted in pygame
        return (round(x_meters, 2), round(y_meters, 2))
    
    def load_background_image(self):
        """Load a background image"""
        file_path = pygame.filedialog.askopenfilename(
            title="Select Background Image",
            filetypes=[("Image files", "*.png;*.jpg;*.jpeg;*.bmp")]
        )
        
        if file_path:
            try:
                self.bg_image = pygame.image.load(file_path)
                self.bg_image = pygame.transform.scale(
                    self.bg_image, 
                    (int(self.field_pixel_width), int(self.field_pixel_height))
                )
                self.bg_image_path = file_path
                print(f"Loaded background image: {file_path}")
            except Exception as e:
                print(f"Error loading background image: {e}")
                self.bg_image = None
    
    def load_navgrid(self):
        try:
            if os.path.exists(self.filename):
                with open(self.filename, 'r') as f:
                    data = json.load(f)
                    
                    # Check if we have a background image path
                    if 'bg_image_path' in data and data['bg_image_path']:
                        try:
                            self.bg_image_path = data['bg_image_path']
                            if os.path.exists(self.bg_image_path):
                                self.bg_image = pygame.image.load(self.bg_image_path)
                                self.bg_image = pygame.transform.scale(
                                    self.bg_image, 
                                    (int(self.field_pixel_width), int(self.field_pixel_height))
                                )
                                print(f"Loaded background image: {self.bg_image_path}")
                        except Exception as e:
                            print(f"Error loading background image: {e}")
                    
                    # Load polygons in meter coordinates
                    if 'polygons' in data:
                        self.polygons = data['polygons']
                    print(f"Loaded {len(self.polygons)} polygons from {self.filename}")
        except Exception as e:
            print(f"Error loading navgrid: {e}")
    
    def save_navgrid(self):
        try:
            data = {
                'polygons': self.polygons,
                'bg_image_path': self.bg_image_path
            }
            with open(self.filename, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"Saved {len(self.polygons)} polygons to {self.filename}")
        except Exception as e:
            print(f"Error saving navgrid: {e}")
    
    def draw_grid(self):
        # Draw field boundary
        field_rect = pygame.Rect(
            self.field_offset_x, 
            self.field_offset_y, 
            self.field_pixel_width, 
            self.field_pixel_height
        )
        pygame.draw.rect(self.screen, (100, 100, 100), field_rect, 2)
        
        # Draw grid lines in meter intervals
        for x_meters in range(int(FIELD_LENGTH) + 1):
            x_pixels, _ = self.meters_to_pixels(x_meters, 0)
            pygame.draw.line(
                self.screen, 
                GRID_COLOR, 
                (x_pixels, self.field_offset_y), 
                (x_pixels, self.field_offset_y + self.field_pixel_height)
            )
            
        for y_meters in range(int(FIELD_WIDTH) + 1):
            _, y_pixels = self.meters_to_pixels(0, y_meters)
            pygame.draw.line(
                self.screen, 
                GRID_COLOR, 
                (self.field_offset_x, y_pixels), 
                (self.field_offset_x + self.field_pixel_width, y_pixels)
            )
    
    def draw_background(self):
        # Draw the background color
        self.screen.fill(BG_COLOR)
        
        # Draw the background image if available
        if self.bg_image:
            self.screen.blit(
                self.bg_image, 
                (self.field_offset_x, self.field_offset_y)
            )
    
    def draw_polygons(self):
        # Draw completed polygons
        for polygon in self.polygons:
            # Convert meter coordinates to pixel coordinates
            pixel_points = [self.meters_to_pixels(point[0], point[1]) for point in polygon['points']]
            
            if len(pixel_points) >= 2:
                pygame.draw.polygon(self.screen, COMPLETE_COLOR, pixel_points, LINE_WIDTH)
                # Draw points
                for point in pixel_points:
                    pygame.draw.circle(self.screen, COMPLETE_COLOR, point, POINT_RADIUS)
        
        # Draw current polygon being created
        if self.is_drawing and len(self.current_polygon) > 0:
            # Convert meter coordinates to pixel coordinates
            pixel_points = [self.meters_to_pixels(point[0], point[1]) for point in self.current_polygon]
            
            # Draw lines between points
            if len(pixel_points) > 1:
                pygame.draw.lines(self.screen, ACTIVE_COLOR, False, pixel_points, LINE_WIDTH)
            
            # Draw points
            for point in pixel_points:
                pygame.draw.circle(self.screen, ACTIVE_COLOR, point, POINT_RADIUS)
            
            # Draw line from last point to mouse position when drawing
            mouse_pos = pygame.mouse.get_pos()
            x_meters, y_meters = self.pixels_to_meters(*mouse_pos)
            
            # Only draw the line if mouse is within field bounds
            if (0 <= x_meters <= FIELD_LENGTH and 0 <= y_meters <= FIELD_WIDTH):
                if len(pixel_points) > 0:
                    pygame.draw.line(self.screen, (255, 255, 255), pixel_points[-1], mouse_pos, 1)
    
    def draw_status(self):
        # Get mouse position in meters
        mouse_pos = pygame.mouse.get_pos()
        x_meters, y_meters = self.pixels_to_meters(*mouse_pos)
        
        status_texts = [
            f"Polygons: {len(self.polygons)}",
            f"Mouse: {x_meters:.2f}m, {y_meters:.2f}m",
            "Q: Start new polygon",
            "Left Click: Add point",
            "E: Complete polygon",
            "S: Save to file",
            "L: Load from file",
            "B: Load background image",
            "Esc: Quit"
        ]
        
        y = 10
        for text in status_texts:
            text_surface = self.font.render(text, True, (255, 255, 255))
            self.screen.blit(text_surface, (10, y))
            y += FONT_SIZE + 5
    
    def run(self):
        running = True
        
        while running:
            self.draw_background()
            self.draw_grid()
            self.draw_polygons()
            self.draw_status()
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    
                    # Start a new polygon with Q
                    elif event.key == pygame.K_q:
                        self.is_drawing = True
                        self.current_polygon = []
                        print("Started new polygon")
                    
                    # Complete the polygon with E
                    elif event.key == pygame.K_e and self.is_drawing:
                        if len(self.current_polygon) >= 3:
                            # Add the polygon to our list
                            self.polygons.append({
                                'id': len(self.polygons),
                                'points': self.current_polygon
                            })
                            self.is_drawing = False
                            print(f"Completed polygon with {len(self.current_polygon)} points")
                        else:
                            print("Need at least 3 points to complete a polygon")
                    
                    # Save with S
                    elif event.key == pygame.K_s:
                        self.save_navgrid()
                    
                    # Load with L
                    elif event.key == pygame.K_l:
                        self.load_navgrid()
                    
                    # Load background with B
                    elif event.key == pygame.K_b:
                        self.load_background_image()
                
                # Add point with left click
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1 and self.is_drawing:  # Left click
                        pixel_pos = pygame.mouse.get_pos()
                        meter_pos = self.pixels_to_meters(*pixel_pos)
                        
                        # Only add point if within field bounds
                        if (0 <= meter_pos[0] <= FIELD_LENGTH and 0 <= meter_pos[1] <= FIELD_WIDTH):
                            self.current_polygon.append(list(meter_pos))
                            print(f"Added point at {meter_pos[0]:.2f}m, {meter_pos[1]:.2f}m")
            
            pygame.display.flip()
            self.clock.tick(60)
        
        pygame.quit()

if __name__ == "__main__":
    # Ensure file dialog works properly
    if not hasattr(pygame, 'filedialog'):
        import tkinter as tk
        from tkinter import filedialog
        pygame.filedialog = filedialog
        root = tk.Tk()
        root.withdraw()  # Hide the main window
    
    editor = NavGridEditor()
    editor.run()
