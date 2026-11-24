/*
 * LedMatrix.cpp - UNO R4 12x8 LED matrix startup/idle animation driver
 *
 * Two-phase looping animation, non-blocking:
 *   1. Reveal - the Analog Devices wedge logo, uncovered row by row.
 *   2. Scroll - the words "Analog Devices" scrolling right to left.
 * Then it repeats.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "LedMatrix.h"

static const uint8_t AD_LOGO[LedMatrix::ROWS][LedMatrix::COLS] = {
    { 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1 },
    { 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1 },
    { 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1 },
};

static constexpr uint8_t GLYPH_WIDTH = 3;
static constexpr uint8_t LETTER_GAP  = 1;

static const uint8_t GLYPH_SPACE[GLYPH_WIDTH] = { 0x00, 0x00, 0x00 };
static const uint8_t GLYPH_A[GLYPH_WIDTH]     = { 0x3C, 0x0A, 0x3C };
static const uint8_t GLYPH_C[GLYPH_WIDTH]     = { 0x1C, 0x22, 0x22 };
static const uint8_t GLYPH_D[GLYPH_WIDTH]     = { 0x3E, 0x22, 0x1C };
static const uint8_t GLYPH_E[GLYPH_WIDTH]     = { 0x3E, 0x2A, 0x22 };
static const uint8_t GLYPH_G[GLYPH_WIDTH]     = { 0x1C, 0x22, 0x3A };
static const uint8_t GLYPH_I[GLYPH_WIDTH]     = { 0x22, 0x3E, 0x22 };
static const uint8_t GLYPH_L[GLYPH_WIDTH]     = { 0x3E, 0x20, 0x20 };
static const uint8_t GLYPH_N[GLYPH_WIDTH]     = { 0x3E, 0x04, 0x3E };
static const uint8_t GLYPH_O[GLYPH_WIDTH]     = { 0x1C, 0x22, 0x1C };
static const uint8_t GLYPH_S[GLYPH_WIDTH]     = { 0x24, 0x2A, 0x12 };
static const uint8_t GLYPH_V[GLYPH_WIDTH]     = { 0x1E, 0x20, 0x1E };

static const char SCROLL_TEXT[] = "Analog Devices";

static uint8_t  colBuf[sizeof(SCROLL_TEXT) * (GLYPH_WIDTH + LETTER_GAP)];
static uint16_t colCount = 0;


static const uint8_t* glyphFor(char c)
{
    if (c >= 'a' && c <= 'z') {
        c = (char)(c - 'a' + 'A');
    }
    switch (c) {
        case 'A': return GLYPH_A;
        case 'C': return GLYPH_C;
        case 'D': return GLYPH_D;
        case 'E': return GLYPH_E;
        case 'G': return GLYPH_G;
        case 'I': return GLYPH_I;
        case 'L': return GLYPH_L;
        case 'N': return GLYPH_N;
        case 'O': return GLYPH_O;
        case 'S': return GLYPH_S;
        case 'V': return GLYPH_V;
        default:  return GLYPH_SPACE;
    }
}

LedMatrix::LedMatrix()
    : phase(Phase::Reveal), frame(0), lastStep(0)
{
}

void LedMatrix::begin()
{
    matrix.begin();
    colCount = 0;
    for (uint16_t i = 0; SCROLL_TEXT[i] != '\0'; i++) {
        const uint8_t* g = glyphFor(SCROLL_TEXT[i]);
        for (uint8_t c = 0; c < GLYPH_WIDTH; c++) {
            colBuf[colCount++] = g[c];
        }
        colBuf[colCount++] = 0x00;
    }

    phase = Phase::Reveal;
    frame = 0;
    lastStep = millis();
    renderReveal(0);
}

void LedMatrix::tick()
{
    unsigned long now = millis();
    unsigned long interval = SCROLL_INTERVAL_MS;
    if (phase == Phase::Reveal) {
        interval = REVEAL_INTERVAL_MS;
    }
    if (now - lastStep < interval) {
        return;
    }
    lastStep = now;

    frame++;

    if (phase == Phase::Reveal) {
        if (frame >= REVEAL_FRAMES) {
            phase = Phase::Scroll;
            frame = 0;
            renderScroll(0);
        } else {
            renderReveal(frame);
        }
    } else {
        uint16_t totalScroll = colCount + COLS; 
        if (frame >= totalScroll) {
            phase = Phase::Reveal;
            frame = 0;
            renderReveal(0);
        } else {
            renderScroll(frame);
        }
    }
}
void LedMatrix::renderReveal(uint16_t f)
{
    uint8_t canvas[ROWS][COLS];

    for (uint8_t row = 0; row < ROWS; row++) {
        for (uint8_t col = 0; col < COLS; col++) {
            if (row < f) {
                canvas[row][col] = AD_LOGO[row][col];
            } else {
                canvas[row][col] = 1;
            }
        }
    }

    matrix.renderBitmap(canvas, ROWS, COLS);
}

void LedMatrix::renderScroll(uint16_t f)
{
    uint8_t canvas[ROWS][COLS];

    for (uint8_t col = 0; col < COLS; col++) {
        int16_t srcCol = (int16_t)f + col - COLS;
        uint8_t bits = 0x00;
        if (srcCol >= 0 && srcCol < (int16_t)colCount) {
            bits = colBuf[srcCol];
        }
        for (uint8_t row = 0; row < ROWS; row++) {
            canvas[row][col] = (bits >> row) & 0x01;
        }
    }

    matrix.renderBitmap(canvas, ROWS, COLS);
}
