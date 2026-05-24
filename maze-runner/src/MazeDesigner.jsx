import React, { useState, useEffect } from 'react';

const MazeDesigner = () => {
    const [width, setWidth] = useState(15);
    const [height, setHeight] = useState(15);
    const [grid, setGrid] = useState([]);
    const [isDrawing, setIsDrawing] = useState(false);

    // 1 = drawing walls (black), 0 = erasing to paths (white)
    const [drawMode, setDrawMode] = useState(1);

    // Initialize and resize grid while preserving existing drawing
    useEffect(() => {
        setGrid(prev => {
            const newGrid = [];
            for (let y = 0; y < height; y++) {
                const row = [];
                for (let x = 0; x < width; x++) {
                    if (prev[y] !== undefined && prev[y][x] !== undefined) {
                        row.push(prev[y][x]);
                    } else {
                        row.push(0); // Default new cells to path
                    }
                }
                newGrid.push(row);
            }
            return newGrid;
        });
    }, [width, height]);

    // Handle global mouse up to stop drawing even if cursor leaves the grid
    useEffect(() => {
        const handleGlobalMouseUp = () => setIsDrawing(false);
        window.addEventListener('mouseup', handleGlobalMouseUp);
        return () => window.removeEventListener('mouseup', handleGlobalMouseUp);
    }, []);

    const handleMouseDown = (y, x) => {
        setIsDrawing(true);
        const newMode = grid[y][x] === 0 ? 1 : 0;
        setDrawMode(newMode);
        updateCell(y, x, newMode);
    };

    const handleMouseEnter = (y, x) => {
        if (isDrawing) {
            updateCell(y, x, drawMode);
        }
    };

    const updateCell = (y, x, value) => {
        setGrid(prev => {
            const newGrid = [...prev];
            newGrid[y] = [...newGrid[y]];
            newGrid[y][x] = value;
            return newGrid;
        });
    };

    const clearGrid = () => {
        setGrid(Array.from({ length: height }, () => Array(width).fill(0)));
    };

    const exportMaze = () => {
        // Convert the 2D array into a string of 1s and 0s separated by newlines
        const textContent = grid.map(row => row.join('')).join('\n');

        // Create a Blob and trigger download
        const blob = new Blob([textContent], { type: 'text/plain' });
        const url = URL.createObjectURL(blob);
        const link = document.createElement('a');
        link.href = url;
        link.download = 'maze_layout.txt';
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
        URL.revokeObjectURL(url);
    };

    return (
        <div className="min-h-screen bg-slate-50 p-8 font-sans text-slate-800">
            <div className="max-w-4xl mx-auto space-y-6">

                {/* Header */}
                <div>
                    <h1 className="text-3xl font-bold text-slate-900">Webots Maze Designer</h1>
                    <p className="text-slate-500 mt-1">Click and drag to draw walls. Export to generate the 3D scene.</p>
                </div>

                {/* Controls Panel */}
                <div className="bg-white p-6 rounded-xl shadow-sm border border-slate-200 grid grid-cols-1 md:grid-cols-2 gap-8">

                    {/* Sliders */}
                    <div className="space-y-4">
                        <div>
                            <div className="flex justify-between mb-1">
                                <label className="text-sm font-medium">Width (Cells)</label>
                                <span className="text-sm font-bold">{width}</span>
                            </div>
                            <input
                                type="range" min="5" max="50" value={width}
                                onChange={(e) => setWidth(parseInt(e.target.value))}
                                className="w-full accent-blue-600"
                            />
                        </div>

                        <div>
                            <div className="flex justify-between mb-1">
                                <label className="text-sm font-medium">Height (Cells)</label>
                                <span className="text-sm font-bold">{height}</span>
                            </div>
                            <input
                                type="range" min="5" max="50" value={height}
                                onChange={(e) => setHeight(parseInt(e.target.value))}
                                className="w-full accent-blue-600"
                            />
                        </div>
                    </div>

                    {/* Actions */}
                    <div className="flex flex-col justify-end space-y-3">
                        <button
                            onClick={clearGrid}
                            className="px-4 py-2 bg-slate-100 hover:bg-slate-200 text-slate-700 font-medium rounded-lg transition-colors"
                        >
                            Clear Grid
                        </button>
                        <button
                            onClick={exportMaze}
                            className="px-4 py-2 bg-blue-600 hover:bg-blue-700 text-white font-medium rounded-lg shadow-sm transition-colors"
                        >
                            Export to Webots (.txt)
                        </button>
                    </div>
                </div>

                {/* Drawing Canvas */}
                <div className="bg-white p-4 rounded-xl shadow-sm border border-slate-200 flex justify-center">
                    <div
                        className="grid gap-[1px] bg-slate-300 border border-slate-300 w-full max-w-3xl select-none cursor-crosshair touch-none"
                        style={{ gridTemplateColumns: `repeat(${width}, minmax(0, 1fr))` }}
                        onMouseLeave={() => setIsDrawing(false)}
                    >
                        {grid.map((row, y) =>
                            row.map((cell, x) => (
                                <div
                                    key={`${x}-${y}`}
                                    onMouseDown={(e) => {
                                        e.preventDefault(); // Prevents default drag-and-drop text behavior
                                        handleMouseDown(y, x);
                                    }}
                                    onMouseEnter={() => handleMouseEnter(y, x)}
                                    className={`
                    aspect-square transition-colors duration-75
                    ${cell === 1 ? 'bg-slate-800' : 'bg-white'} 
                    hover:opacity-75
                  `}
                                />
                            ))
                        )}
                    </div>
                </div>

            </div>
        </div>
    );
};

export default MazeDesigner;