document.addEventListener('DOMContentLoaded', () => {
    const mainContentWrapper = document.querySelector('.main-content-wrapper');
    const syllabusContainer = document.querySelector('.syllabus-container');
    const chaptersContainer = document.querySelector('.chapters-container');
    const backButton = document.querySelector('.back-button');
    const chapterSlidesWrapper = document.querySelector('.chapter-slides-wrapper');

    const modulesData = {
        '1': ['Chapter 1.1: Intro', 'Chapter 1.2: Core Concepts', 'Chapter 1.3: Advanced Topics', 'Chapter 1.4: Summary'],
        '2': ['Chapter 2.1: Getting Started', 'Chapter 2.2: Deep Dive'],
        '3': ['Chapter 3.1', 'Chapter 3.2', 'Chapter 3.3'],
        '4': ['Chapter 4.1'],
        '5': ['Chapter 5.1', 'Chapter 5.2', 'Chapter 5.3', 'Chapter 5.4', 'Chapter 5.5'],
        '6': ['Chapter 6.1', 'Chapter 6.2'],
        '7': ['Chapter 7.1', 'Chapter 7.2', 'Chapter 7.3'],
        '8': ['Chapter 8.1', 'Chapter 8.2'],
    };

    // Use event delegation for module clicks
    if (syllabusContainer) {
        syllabusContainer.addEventListener('click', (e) => {
            const moduleSlide = e.target.closest('.syllabus-slide');
            if (moduleSlide) {
                const moduleId = moduleSlide.dataset.module;
                if (moduleId) {
                    openChaptersView(moduleId);
                }
            }
        });
    }

    if (backButton) {
        backButton.addEventListener('click', () => {
            closeChaptersView();
        });
    }

    function openChaptersView(moduleId) {
        const chapters = modulesData[moduleId] || [];
        
        // Clear previous chapters and generate new ones
        chapterSlidesWrapper.innerHTML = '';
        if (chapters.length > 0) {
            chapters.forEach(chapterTitle => {
                const slide = document.createElement('div');
                slide.className = 'chapter-slide';
                slide.textContent = chapterTitle;
                chapterSlidesWrapper.appendChild(slide);
            });
        } else {
            chapterSlidesWrapper.innerHTML = '<p class="no-chapters">No chapters available for this module.</p>';
        }

        mainContentWrapper.classList.add('blurred');
        chaptersContainer.classList.remove('hidden');
    }

    function closeChaptersView() {
        mainContentWrapper.classList.remove('blurred');
        chaptersContainer.classList.add('hidden');
    }
});
